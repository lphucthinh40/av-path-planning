#include "pathplanner.h"

// CONSTRUCTOR
PathPlanner::PathPlanner(int return_size, double range, vector<double> &in_map_s, vector<double> &in_map_x, vector<double> &in_map_y)
{
	  map_s = in_map_s;
	  map_x = in_map_x;
	  map_y = in_map_y;
	  path_return_size = return_size;
	  sensor_range = range;
}

// DESTRUCTOR
PathPlanner::~PathPlanner() {}

/**
 * Update PathPlaner based on feedback data
 *
 * @param ego_vehicle Current status of ego_vehicle.
 * @param sensor_data Raw sensor data from simulator.
 * @param n_consumed_waypoints Number of waypoints consumed from the previous update
 */
void PathPlanner::update(Vehicle &ego_vehicle, vector<vector<double>> &sensor_data, int n_consumed_waypoints)
{
	ego = ego_vehicle;

	// get traffic report
	traffic = getTrafficReport(sensor_data);

	// discard consumed waypoints
	if (proposed_path.size()>0)
	{
		proposed_path.erase(proposed_path.begin(), proposed_path.begin() + n_consumed_waypoints);
		ego.st = proposed_path[0].st;
	}

	// check if ego is in the best lane
	int best_lane = getBestLane();

	// require at least 1 second delay between continuous lane changing attempts
	if ((counter != 0) && ego.st == CS)
		counter = (counter+1) % 50;

	// lane change right
	if ((ego.lane < best_lane) && (ego.st == CS) && isLaneChangeSafe(false) && (counter==0))
	{
		proposed_path = CL_Trajectory(false);
		counter++;
	}
	// lane change left
	else if ((ego.lane > best_lane) && (ego.st == CS) && isLaneChangeSafe(true) && (counter==0))
	{
		proposed_path = CL_Trajectory(true);
		counter++;
	}
	// keep lane
	else
		proposed_path = KL_Trajectory();
}

/**
 * Get trajectory as x & y vectors that can be used externally
 *
 * @return {x_vector, y_vector} Lists of separated x & y values of the latest trajectory.
 */
vector<vector<double>> PathPlanner::getPath()
{
	vector<double> xs, ys;
	for (int i=0; i<path_return_size; ++i)
	{	xs.push_back(proposed_path[i].x);
		ys.push_back(proposed_path[i].y);
	}
	return {xs, ys};
}

// __________ TRAJECTORY GENERATORS ____________

/**
 * Lane Change Trajectory Generator
 * change lane while maintaining constant speed
 *
 * @param change_left True->left_change, false->right_change
 * @return path Newly generated trajectory
 */
vector<Vehicle> PathPlanner::CL_Trajectory(bool change_left)
{
	double ego_coor_x, ego_coor_y;
	vector<Vehicle> path;
	Vehicle temp_vehicle = ego;
	temp_vehicle.st = (change_left)? LCL: LCR;
	temp_vehicle.a = 0;
	temp_vehicle.d = ego.lane*4+2;
	double prev_x = ego.x;
	double prev_y = ego.y;

	// Generate reference points for Spline
	vector<vector<double>> xy_anchors = getLaneChangePoints(change_left);

	// Initialize Spline
	tk::spline s;
	s.set_boundary(s.first_deriv, 0, s.first_deriv, 0, false);
	s.set_points(xy_anchors[0], xy_anchors[1]);

	// Generate 30 extra points to prevent premature lane-change trajectory
	 for (int i=1; i<=path_return_size+30; ++i)
	{
		// x & y in vehicle coordinates
    	ego_coor_x = i*TIME_STEP*ego.v;
    	ego_coor_y = s(ego_coor_x);
    	// x & y in world coordinates
		temp_vehicle.x = ego_coor_x * cos(ego.yaw) - ego_coor_y * sin(ego.yaw) + ego.x;
		temp_vehicle.y = ego_coor_x * sin(ego.yaw) + ego_coor_y * cos(ego.yaw) + ego.y;

		// update yaw, frenet coordinates & lane number
		temp_vehicle.yaw = atan2(temp_vehicle.y - prev_y, temp_vehicle.x - prev_x);
		vector<double> frenet = getFrenet(temp_vehicle.x, temp_vehicle.y, temp_vehicle.yaw, map_x, map_y);
		temp_vehicle.s = frenet[0];
		temp_vehicle.d = frenet[1];
		temp_vehicle.lane = temp_vehicle.d / 4;

		// add to path
		path.push_back(temp_vehicle);

		// update prev_x, prev_y for calculating yaw
		prev_x = temp_vehicle.x;
		prev_y = temp_vehicle.y;
	}
	return path;
}


/**
 * Keep Lane Trajectory Generator
 * maintain constant speed & change speed if necessary
 *
 * @return path Newly generated trajectory
 */
vector<Vehicle> PathPlanner::KL_Trajectory()
{
	vector<Vehicle> new_path = proposed_path;
	vector<Vehicle> added_path;
	Vehicle ref_vehicle;

	// size of the current internal trajectory
	int n = proposed_path.size();

	// get latest ego-vehicle state
	if (n == 0)	ref_vehicle = ego;
	else 		ref_vehicle = proposed_path.back();

	// check if there is a vehicle ahead. If so, match up its speed.
	// otherwise, maintain OPTIMAL_SPEED as often as possible
	if (traffic.vehicle_ahead[ref_vehicle.lane].id != -1)
	{	target_speed = traffic.vehicle_ahead[ref_vehicle.lane].v;
	}
	else
		target_speed = OPTIMAL_SPEED;

	// only insert new points if the current internal trajectory drops below return_path_size
	if (n < path_return_size)
	{
		// use constant speed projection if current speed matches target speed.
		// otherwise, use scurve projection to change speed
		if ((abs(ref_vehicle.v-target_speed)<1.0))
		{	ref_vehicle.v = target_speed;
			added_path =  KL_Constant_Trajectory(ref_vehicle, path_return_size - n);
		}
		else
			added_path = KL_Scurve_Trajectory(ref_vehicle, target_speed);
	}

	// insert new points to the current internal trajectory
	new_path.insert(new_path.end(), added_path.begin(), added_path.end());

	return new_path;
}

/**
 * Keep Lane CONSTANT SPEED Trajectory Generator
 * maintain constant speed & follow current lane
 *
 * @param start starting state of ego-vehicle
 * @parem n_timestep number of future point to generate
 * @return path Newly generated trajectory
 */
vector<Vehicle> PathPlanner::KL_Constant_Trajectory(Vehicle start, int n_timestep)
{
	double ego_coor_x, ego_coor_y;
	vector<Vehicle> path;
	Vehicle temp_vehicle = start;
	temp_vehicle.st = State::CS;
	temp_vehicle.a = 0;
	temp_vehicle.d = start.lane*4+2; // force d to be in center of lane
	double prev_x = start.x;
	double prev_y = start.y;

	// generate reference points for Spline
	vector<vector<double>> xy_anchors = getAnchorPoints();

	// initialize Spline
	tk::spline s;
	s.set_points(xy_anchors[0], xy_anchors[1]);

    for (int i=1; i<=n_timestep; ++i)
	{
    	// x, y in vehicle coordinates
    	ego_coor_x = i*TIME_STEP*start.v;
    	ego_coor_y = s(ego_coor_x);
		// x, y in world coordinates
		temp_vehicle.x = ego_coor_x * cos(start.yaw) - ego_coor_y * sin(start.yaw) + start.x;
		temp_vehicle.y = ego_coor_x * sin(start.yaw) + ego_coor_y * cos(start.yaw) + start.y;

		// update yaw, frenet coordinates
		temp_vehicle.yaw = atan2(temp_vehicle.y - prev_y, temp_vehicle.x - prev_x);
		vector<double> frenet = getFrenet(temp_vehicle.x, temp_vehicle.y, temp_vehicle.yaw, map_x, map_y);
		temp_vehicle.s = frenet[0];

		// add to path
		path.push_back(temp_vehicle);

		// update prev_x, prev_y for calculating yaw
		prev_x = temp_vehicle.x;
		prev_y = temp_vehicle.y;
	}
	return path;
}

/**
 * Keep Lane CHANGE SPEED Trajectory Generator
 * change to target speed as quickly as possible using S-Curve
 *
 * @param start starting state of ego-vehicle
 * @parem target_v target speed
 * @return path Newly generated trajectory
 */
vector<Vehicle> PathPlanner::KL_Scurve_Trajectory(Vehicle start, double target_v)
{
	double ego_coor_x, ego_coor_y;
	double t=0;
	vector<Vehicle> path;
	Vehicle temp_vehicle = start;
	temp_vehicle.st = State::KL;
	temp_vehicle.d = start.lane*4+2; // force d to be in center of lane
	vector<double> xy;
	double prev_x = start.x;
	double prev_y = start.y;

	// initialize S-Curve generator
	SCurveGenerator scg(start.v, target_v, ACCEL_MAX, JERK_MAX);

	// generate reference points for Spline
	vector<vector<double>> xy_anchors = getAnchorPoints();

	// initialize Spline
	tk::spline s;
	s.set_points(xy_anchors[0], xy_anchors[1]);

	// scg.T is the time it takes to reach target speed
	while (t<=scg.T-TIME_STEP)
	{
		t += TIME_STEP;

		// x, y in vehicle coordinates
		ego_coor_x = scg.s(t);
		ego_coor_y = s(ego_coor_x);

		// x, y in world coordinates
		temp_vehicle.x = ego_coor_x * cos(start.yaw) - ego_coor_y * sin(start.yaw) + start.x;
		temp_vehicle.y = ego_coor_x * sin(start.yaw) + ego_coor_y * cos(start.yaw) + start.y;

		// update yaw, Frenet coordinates, speed & acceleration
		temp_vehicle.yaw = atan2(temp_vehicle.y - prev_y, temp_vehicle.x - prev_x);
		vector<double> frenet = getFrenet(temp_vehicle.x, temp_vehicle.y, temp_vehicle.yaw, map_x, map_y);
		temp_vehicle.s = frenet[0];
		temp_vehicle.v = scg.v(t);
		temp_vehicle.a = scg.a(t);

		// add to path
		path.push_back(temp_vehicle);

		// update prev_x, prev_y for calculating yaw
		prev_x = temp_vehicle.x;
		prev_y = temp_vehicle.y;
	}

	// add an extra timestep where target_speed is already reached and acceleration becomes zero
	temp_vehicle.v = scg.vs;
	temp_vehicle.a = 0;
	ego_coor_x = ego_coor_x + temp_vehicle.v*TIME_STEP;
	ego_coor_y = s(ego_coor_x);
	temp_vehicle.x = ego_coor_x * cos(start.yaw) - ego_coor_y * sin(start.yaw) + start.x;
	temp_vehicle.y = ego_coor_x * sin(start.yaw) + ego_coor_y * cos(start.yaw) + start.y;
	temp_vehicle.yaw = atan2(temp_vehicle.y - prev_y, temp_vehicle.x - prev_x);
	vector<double> frenet = getFrenet(temp_vehicle.x, temp_vehicle.y, temp_vehicle.yaw, map_x, map_y);
	temp_vehicle.s = frenet[0];
	temp_vehicle.d = (int) frenet[1];
	path.push_back(temp_vehicle);

	return path;
}

// ___________ HELPER FUNCTIONS _____________

/**
 * Keep track of nearby vehicles within range
 *
 * @param sensor_data Raw sensor data from simulator
 * @return traffic A report of vehicles ahead & behind ego-vehicle on the three lanes
 */
Traffic PathPlanner::getTrafficReport(vector<vector<double>> &sensor_data)
{	  // [ id, x, y, vx, vy, s, d]
  Traffic traffic;
  for (int i=0; i<3; ++i)
  { traffic.vehicle_ahead[i].id = -1;
  	traffic.vehicle_behind[i].id = -1;
  }

  double s, distance, dummy_distance;
  int lane;
  bool is_ahead;

  // go throught list of all other vehicles
  // keep track of those closest to ego-vehicle from each lane, ahead & behind
  for (int i=0; i<sensor_data.size(); ++i)
  {	  s = sensor_data[i][5];
	  lane = sensor_data[i][6] / 4;
	  is_ahead = isAhead(ego.s, s, distance);

	  if (distance <= sensor_range)
	  {	if (is_ahead)
		{ if (traffic.vehicle_ahead[lane].id == -1)
			  traffic.vehicle_ahead[lane] = convert2Vehicle(sensor_data[i]);
		  else if (isAhead(s, traffic.vehicle_ahead[lane].s, dummy_distance))
			  traffic.vehicle_ahead[lane] = convert2Vehicle(sensor_data[i]);
		}
		else
		{ if (traffic.vehicle_behind[lane].id == -1)
			  traffic.vehicle_behind[lane] = convert2Vehicle(sensor_data[i]);
		  else if (isAhead(traffic.vehicle_behind[lane].s, s, dummy_distance))
			  traffic.vehicle_behind[lane] = convert2Vehicle(sensor_data[i]);
		}
	  }
   }
   return traffic;
}

/**
 * Check if Vehicle_2 is ahead of behind of Vehicle_1 based on their s values
 * Also return the relative distance between the two
 *
 * @param s1 longitude of vehicle 1 in Frenet
 * @param s2 longitude of vehicle 2 in Frenet
 * @return true if vehicle 2 is ahead of vehicle 1, false if vehicle 2 is behind vehicle 1
 * @return relative_distance(via pass-by-reference) relative distance between s1 to s2 (always positive)
 */
bool PathPlanner::isAhead(double s1, double s2, double &relative_distance)
{	  double ahead_distance, behind_distance;

  if (s2>=s1)
  {	ahead_distance = s2 - s1;
	behind_distance = MAP_MAX_S - ahead_distance;
  }
  else
  { behind_distance = s1 - s2;
	ahead_distance = MAP_MAX_S - behind_distance;
  }

  if (ahead_distance <= behind_distance)
  {	relative_distance = ahead_distance;
	return true;
  }
  else
  { relative_distance = behind_distance;
	return false;
  }
}

/**
 * Convert a vector<double> to Vehicle structure
 *
 * @param raw_data a vector<double> that stores vehicle state
 * @return A converted Vehicle structure
 */
Vehicle PathPlanner::convert2Vehicle(vector<double> &raw_data)
{		return (Vehicle) { (int) raw_data[0], // id
			 raw_data[1], // x
			 raw_data[2], // y
			 raw_data[5], // s
			 raw_data[6], // d
			 (int)raw_data[6]/4, // lane
			 atan2(raw_data[4], raw_data[3]), // yaw
			 sqrt(pow(raw_data[3],2) + pow(raw_data[4],2)), // v
			 0,   // a
			 0,   // j
			 NONE // state
		   };
}

/**
 * Generate anchor points for Spline in keep lane projection
 *
 * @param use_current_ego(boolean) 	true->start at current ego
 * 									false->start at the last waypoint of the internal trajectory
 * @return x vector & y vector of referenced points
 */
vector<vector<double>> PathPlanner::getAnchorPoints(bool use_current_ego)
{
	Vehicle ref_ego;
	Vehicle prev_ref_ego;

	// Check if proposed path is available
	vector<double> xs;
	vector<double> ys;
	int n;

	if (use_current_ego) n = 0;
	else				 n = proposed_path.size();

	// Find referenced Vehicle
	if (n>0) ref_ego = proposed_path[n-1];
	else     ref_ego = ego;

	// Find previous state of referenced Vehicle
	if 		(n-1>0)  prev_ref_ego = proposed_path[n-2];
	else if (n-1==0) prev_ref_ego = ego;
	else
	{	vector<double> raw_state = {0, ref_ego.x - cos(ref_ego.yaw), ref_ego.y - sin(ref_ego.yaw),
									0,0,0,0,0,0,0, NONE};
		prev_ref_ego = convert2Vehicle(raw_state);
	}

	// Add both to anchor list
	xs.push_back(prev_ref_ego.x);
	xs.push_back(ref_ego.x);
	ys.push_back(prev_ref_ego.y);
	ys.push_back(ref_ego.y);

	// add three more future waypoints
	vector<double> wp1 = getXY(ref_ego.s+20, ref_ego.lane*4+2, map_s, map_x, map_y);
	vector<double> wp2 = getXY(ref_ego.s+40, ref_ego.lane*4+2, map_s, map_x, map_y);
	vector<double> wp3 = getXY(ref_ego.s+60, ref_ego.lane*4+2, map_s, map_x, map_y);

	// Add them to anchor list
    xs.push_back(wp1[0]);
    xs.push_back(wp2[0]);
    xs.push_back(wp3[0]);
    ys.push_back(wp1[1]);
    ys.push_back(wp2[1]);
    ys.push_back(wp3[1]);

//    printf("RAW\n", xs.size());
//	for (int i=0; i<xs.size(); ++i)
//    {  	printf("x[%d]: %f, y[%d]: %f\n", i, xs[i], i, ys[i]);
//    }

    // Convert all to vehicle coordinate
    for (int i=0; i<xs.size(); ++i)
    {
    	double shift_x = xs[i] - ref_ego.x;
    	double shift_y = ys[i] - ref_ego.y;

    	xs[i] =  shift_x * cos(ref_ego.yaw) + shift_y * sin(ref_ego.yaw);
    	ys[i] = -shift_x * sin(ref_ego.yaw) + shift_y * cos(ref_ego.yaw);
    }

  //   if(use_current_ego)
  //   {    printf("n: %d\n", xs.size());
		// for (int i=0; i<xs.size(); ++i)
		// 	printf("x[%d]: %f, y[%d]: %f\n", i, xs[i], i, ys[i]);
  //   }

    return {xs, ys};
}

/**
 * Generate anchor points for Spline in lane change projection
 *
 * @param left_change(boolean) 	true->change left
 * 								false->change right
 * @return x vector & y vector of referenced points
 */
vector<vector<double>> PathPlanner::getLaneChangePoints(bool left_change)
{
	Vehicle ref_ego;
	Vehicle prev_ref_ego;
	Vehicle next_ref_ego;
	float offset = (left_change)? -1:1;
	// Check if proposed path is available
	vector<double> xs;
	vector<double> ys;

	ref_ego = ego;

	vector<double> raw_state = {0, ref_ego.x - cos(ref_ego.yaw), ref_ego.y - sin(ref_ego.yaw),
									0,0,0,0,0,0,0, NONE};
	prev_ref_ego = convert2Vehicle(raw_state);

	raw_state = {0, ref_ego.x + cos(ref_ego.yaw), ref_ego.y + sin(ref_ego.yaw),
									0,0,0,0,0,0,0, NONE};
	next_ref_ego = convert2Vehicle(raw_state);

	// Add both to anchor list
	xs.push_back(prev_ref_ego.x);
	xs.push_back(ref_ego.x);
	xs.push_back(next_ref_ego.x);
	ys.push_back(prev_ref_ego.y);
	ys.push_back(ref_ego.y);
	ys.push_back(next_ref_ego.y);

	// add three more future waypoints
	double future_s = traffic.vehicle_ahead[ref_ego.lane].s;
	vector<double> wp1 = getXY(future_s, (ref_ego.lane+offset)*4+2, map_s, map_x, map_y);
	vector<double> wp2 = getXY(future_s+30, (ref_ego.lane+offset)*4+2, map_s, map_x, map_y);
	vector<double> wp3 = getXY(future_s+60, (ref_ego.lane+offset)*4+2, map_s, map_x, map_y);


	// Add them to anchor list
    xs.push_back(wp1[0]);
    xs.push_back(wp2[0]);
    xs.push_back(wp3[0]);
    ys.push_back(wp1[1]);
    ys.push_back(wp2[1]);
    ys.push_back(wp3[1]);

    // Convert all to vehicle coordinate
    for (int i=0; i<xs.size(); ++i)
    {
    	double shift_x = xs[i] - ref_ego.x;
    	double shift_y = ys[i] - ref_ego.y;

    	xs[i] =  shift_x * cos(ref_ego.yaw) + shift_y * sin(ref_ego.yaw);
    	ys[i] = -shift_x * sin(ref_ego.yaw) + shift_y * cos(ref_ego.yaw);
    }

    return {xs, ys};
}

/**
 * Get best lane based on traffic report and current ego-vehicle state
 *
 * @return target_lane Lane where traffic moves at the highest speed
 */
int PathPlanner::getBestLane()
{
	int target_lane = -1;
	double lane_speed = -1;

	// stay in lane if there is no vehicle ahead
	if  (traffic.vehicle_ahead[ego.lane].id == -1)
		return ego.lane;
	// otherwise, check other lane
	else
	{
		for (int i=0; i<3; ++i)
		{	// if there is a free lane, it's clearly the best (LCL preferred)
			if (traffic.vehicle_ahead[i].id == -1)
				return i;
			double new_lane_speed = traffic.vehicle_ahead[i].v;
			if (lane_speed < new_lane_speed)
			{	lane_speed = new_lane_speed;
				target_lane = i;
			}
		}
	}
	return target_lane;
}

/**
 * Check if it is safe to perform a lane change
 * Lane change is only safe when there is enough gap between ego_vehicle and other vehicles
 * in the target lane
 *
 * @return target_lane Lane where traffic moves at the highest speed
 */
bool PathPlanner::isLaneChangeSafe(bool left_change)
{	int target_lane = (left_change)? ego.lane-1:ego.lane+1;
	if ((target_lane<0) || (target_lane>2))
		return false;
	bool car_ahead = (traffic.vehicle_ahead[target_lane].id != -1);
	bool car_behind = (traffic.vehicle_behind[target_lane].id != -1);
	double rel_distance;

	if (car_ahead)
	{	isAhead(ego.s, traffic.vehicle_ahead[target_lane].s, rel_distance);
		if (rel_distance<20)
			return false;
		if (car_behind)
		{	isAhead(ego.s, traffic.vehicle_behind[target_lane].s, rel_distance);
			if (rel_distance<8)	return false;
			else 					return true;
		}
	}
	else if (car_behind)
	{	isAhead(ego.s, traffic.vehicle_behind[target_lane].s, rel_distance);
		if (rel_distance<8)		return false;
		else 						return true;
	}

	return true;
}
