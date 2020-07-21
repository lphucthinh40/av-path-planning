[//]: # (Image References)
[image_0]: ./motion_profile.png "S-Curve Motion Profile"

# Highway Path Planning Project :oncoming_automobile:
Using Term-3 Simulator - Self-Driving Car Engineer Nanodegree Program 

**Objective:** <br>
The goal of this project is to drive a vehicle autonomously on a freeway simulated environment. The vehicle should not violate speed limit (50mph)or exceed maximum acceleration (10m/s<sup>2</sup>) and maximum jerk (10m/s<sup>3</sup>). In addition, the vehicle should be able to adjust its speed and change lane if necessary to avoid collision with other vehicles.  

---

## Build Instruction
Direct build:

    mkdir build && cd build
    cmake .. && make
    ./path_planning

Using Eclipse:

    cmake -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j8
    import this project to Eclipse

NOTE: Please make sure your path to "highway_map.csv" is correct in main.cpp

---

## Design & Implementation

### 1. Design of PathPlanner Class

PathPlanner is created as the core planning & execution unit for ego-vehicle. It stores the current state of ego-vehicle and nearby traffics along with the projected trajectory. Whenever a new feedback data is collected from the simulator, PathPlanner discards consumed waypoints and uses trajectory generators to add new waypoints to the projected trajectory. Internally, there is no restriction on how many future waypoints PathPlanner can store (in fact, PathPlanner stores over 100 future waypoints to transition smoothly & efficiently from 0 to 50mph).

**APIs**:<br>
- PathPlanner.update(): update the current state of ego-vehicle as well as nearby traffics; determine which trajectory generator to be used to refill the future path.<br>
- PathPlanner.getPath(): get x & y vectors of future path that can be send to the simulator.

**Internal Structures:**<br>
- Vehicle: store the state of a vehicle at a specific timestep to the future. <br>
- Traffic: store references to nearby vehicles. PathPlanner has a sensor range (which is set to 40 meters in main.cpp). Traffic report will only keep track of vehicles that are the closests to ego-vehicle in each lane (one ahead & one from behind) and within sensor range. 

### 2. Trajectory Generators

There are two core trajectory generators: **KL_Trajectory** (for keeping lane) & **CL_Trajectory** (for changing lane). Both generators use **Spline.h** (cubic-square interpolation) to generate a smooth projected path. There are also two helper functions **getAnchorPoints()** & **getLaneChangePoints()**, which are used to generate reference points (in vehicle coordinates) needed to initialize Spline. The curve generated by Spline is guaranteed to pass through these referenced points. As a result, if we continously pass a set future waypoints (where ego-vehicle should pass through), Spline will give us an smooth trajectory in vehicle coordinates. Note that KL_Trajectory only adds new future waypoints when the internal projected trajectory has less than 50 waypoints. Also, whenever CL_Trajectory (change lane) is called, the internal trajectory will be discarded and replaced by a new one, where initial position will be the current position of ego-vehicle.

For **KL_Trajectory**, there are two more sub-trajectories: **KL_Constant_Trajectory** & **KL_Scurve_Trajectory**. KL_Constant_Trajectory is used for driving with constant speed & Scurve_Trajectory is used for changing speed. KL_Scurve_Trajectory is often triggered when ego-vehichle needs to match up the speed of the vehicle in front of it. It is also used to reach optimal speed when the current lane is open. In addition, along with using Spline to generate future trajectory, KL_Scurve_Trajectory also uses Scurve to generate future displacement, velocity, and acceleration.

How a trajectory generator is chosen:

- PathPlanner.update() is called
- Current ego state & traffic report are updated 
- Fastest lane is determined using getBestLane()
- If ego is not in the fastest lane and it is safe to change lane (checking via isLaneChangeSafe()), **CL_Trajectory** is chosen.
- Otherwise, **KL_Trajectory** is chosen.
    + If there is a vehicle ahead, target_speed is set to that vehicle's speed. Otherwise, target_speed is set to Optimal Speed (48mph).
    + If ego's current speed matches with target speed, **KL_Constant_Trajectory** is used internally.
    + Otherwise, **KL_Scurve_Trajectory** is used internally to adjust ego's speed to target_speed.

For **isLaneChangeSafe()**, lane change is considered safe when the vehicle ahead & behind (in target lane) is at least 20 meters & 8 meters away from ego-vehicle, respectively.

For **getBestLane()**, the best lane is determined as follow:

- If current lane is clear, current lane is the best lane.
- If current lane is blocked, check all three lanes from left to right. If a lane is clear, that lane is the best lane (left change is prioritized).
- If all three lanes are blocked, lane that has the highest speed is the best lane.


### 3. Motion Profile Generator

In order to help ego-vehicle change speed quickly & efficiently without exceeding maximum jerk & acceleration, S-Curve Motion Profile is implemented in scurve.cpp. SCurve is initialized with initial speed, target speed, acceleration & jerk limit. It generates a motion profile for adjusting speed, maximizing the period where the vehicle is at acceleration limit without violating jerk limit. 'SCurve.T' gives us the period of speed change. Displacement, velocity, and acceleration can be calculated at any given time within this period using 'Scurve.s(t)', 'Scurve.v(t)', 'Scurve.a(t)' respectively. My implementation is based on "CONSTANT JERK EQUATIONS FOR A TRAJECTORY GENERATOR" BYU.EDU lecture note (http://www.et.byu.edu/~ered/ME537/Notes/Ch5.pdf).

This approach makes it possible to speed up ego-vehicle from 0 to 50mph in nearly 3 seconds without violating jerk & acceleration limits. 

![alt text][image_0]

## Demo (Youtube)

[![](http://img.youtube.com/vi/I3dN14Lfd8o/0.jpg)](http://www.youtube.com/watch?v=I3dN14Lfd8o "")






