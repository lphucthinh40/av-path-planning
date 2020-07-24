/**
    Highway Path Planner
    @author Thinh Lu
    @version 1.0 07/19/20
*/

#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <algorithm>

#include "spline.h"
#include "helpers.h"
#include "scurve.h"

// PATHPLANNER CONSTANTS
#define N_RETURN_WAYPOINTS 	50
#define SENSOR_RANGE	      50
#define OPTIMAL_SPEED       21.5
#define MIN_DISTANCE_AHEAD  30
#define MIN_DISTANCE_BEHIND 10
#define MAX_SPEED_DIFFERENCE 3
#define SPEED_CHANGE_DISTANCE 40
#define CURVATURE_THRESHOLD 0.007

// CONSTRAINT CONSTANTS
#define SPEED_LIMIT 		22.352
#define ACCEL_MAX 			9
#define JERK_MAX 			  9
#define TIME_STEP 			0.02

// MAP CONSTANTS
#define LANE_WIDTH 			4
#define NUMBER_OF_LANE 	3
#define MAP_MAX_S 			6945.554

using std::vector;

// Ego-vehicle States
enum State {CS=0, KL=1, LCL=2, LCR=3, NONE=4};

// Structure to store vehicle state in each time step
struct Vehicle {
	int id;
	double x;
	double y;
	double s;
	double d;
	int lane;
	double yaw;
	double v;
	double a;
	double j;
	State st;
};

// Structure to track nearby cars around ego-vehicle
struct Traffic {
	Vehicle vehicle_ahead[3];
	Vehicle vehicle_behind[3];
};

// --------- PATHPLANNER CLASS ---------
class PathPlanner {

  public:

  // Constructors
  PathPlanner(int return_size, double range,
              vector<double> &in_map_s, vector<double> &in_map_x, vector<double> &in_map_y);
  virtual ~PathPlanner();

  // APIs
  void update(Vehicle &ego_vehicle, vector<vector<double>> &sensor_data, int n_consumed_waypoints);
  vector<vector<double>> getPath();

  // Trajectory Generators
  vector<Vehicle> CL_Trajectory(bool change_left);
  vector<Vehicle> KL_Trajectory();

  // Sub-trajectories for KeepLane state
  vector<Vehicle> KL_Constant_Trajectory(Vehicle start, int n_timestep);  // constant speed
  vector<Vehicle> KL_Scurve_Trajectory(Vehicle start, double target_v);   // for reaching target speed (fastest)

  // Helper Functions
  Traffic getTrafficReport(vector<vector<double>> &sensor_data);
  bool isAhead(double s1, double s2, double &relative_distance);
  double getRelativeDistance(double s1, double s2);
  Vehicle convert2Vehicle(vector<double> &raw_data);
  vector<vector<double>> getAnchorPoints(bool use_current_ego=false);
  vector<vector<double>> getLaneChangePoints(bool left_change);
  int getBestLane();
  bool isLaneChangeSafe(bool left_change);

   // Containers
  vector<Vehicle> proposed_path;
  Traffic traffic;

  // Map Waypoints
  vector<double> map_s;
  vector<double> map_x;
  vector<double> map_y;

  // Properties
  int counter = 0;
  bool allow_lanechange = true;
  int path_return_size;
  double sensor_range;   // meter
  double target_speed;
  Vehicle ego;
};

#endif
