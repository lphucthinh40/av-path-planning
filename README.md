## Highway Path Planning Project :oncoming_automobile:
Using Term-3 Simulator - Self-Driving Car Engineer Nanodegree Program 

**Objective:** <br>
The goal of this project is to drive a vehicle autonomously on a freeway simulated environment. The vehicle should not violate speed limit (50mph)or exceed maximum acceleration (10m/s<sup>2</sup>) and maximum jerk (10m/s<sup>3</sup>). In addition, the vehicle should be able to adjust its speed and change lane if necessary to avoid collision with other vehicles.  

---

#### Build Instruction
Direct build:

    mkdir build && cd build
    cmake .. && make
    ./path_planning

Using Eclipse:

    cmake -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j8
    import this project to Eclipse

NOTE: Please make sure your path to "highway_map.csv" is correct in main.cpp

---

#### Design & Implementation

**1. Design of PathPlanner Class**

PathPlanner is created as the core planning & execution unit for ego-vehicle. It stores the current state of ego-vehicle and nearby traffics along with the projected trajectory. Whenever a new feedback data is collected from the simulator, PathPlanner discards consumed waypoints and uses trajectory generators to add new waypoints to the projected trajectory. Internally, there is no restriction on how many future waypoints PathPlanner can store (in fact, PathPlanner stores over 100 future waypoints to transition smoothly & efficiently from 0 to 50mph). However, PathPlanner always return 50 future waypoints to move ego-vehicle forward on the simulator.   

**2. Trajectory Generators**

There are two core trajectory generators: KeepLane_Trajectory & ChangeLane_Trajectory. Both generators use Spline.h (cubic-square interpolation) to generate a smooth projected path. There are also two helper functions getAnchorPoints() & getLaneChangePoints(), which are used to generate reference points (in vehicle coordinates) needed to initialize Spline.
Spline is used to map x to y in vehicle coordinates, as a result, we must convert x, y back to global coordinates after using it.

For KeepLane_Trajectory, there are two more sub-trajectories: Constant_Trajectory & Scurve_Trajectory. Constant_Trajectory is used for driving with constant speed & Scurve_Trajectory is used for changing speed. Scurve_Trajectory is often triggered when ego-vehichle needs to match up the speed of the vehicle in front of it. It is also used to reach optimal speed when the current lane is open. On the other hand, ChangeLane_Trajectory is developed the same way as Constant_Trajectory. The only difference is how they construct the reference points for Spline initialization.

Step by step on how a trajectory generator is chosen:

- PathPlanner.update() is called
- Current ego state & traffic report are updated 
- Fastest lane is determined using getBestLane()
- If ego is not in the fastest lane and it is safe to change lane (checking via isLaneChangeSafe()), **ChangeLane_Trajectory** is chosen.
- Otherwise, **KeepLane_Trajectory** is chosen.
    + If there is a vehicle ahead, target_speed is set to that vehicle's speed. Otherwise, target_speed is set to Optimal Speed (48mph).
    + If ego's current speed matches with target speed, **Constant_Trajectory** is used internally.
    + Otherwise, **Scurve_Trajectory** is used internally to adjust ego's speed to target_speed.


**3. Motion Profile Generator**

In order to help ego-vehicle change speed quickly & efficiently without exceeding maximum jerk & acceleration, S-Curve Motion Profile is implemented in scurve.cpp. This approach makes it possible to speed up ego-vehicle from 0 to 50mph in nearly 3 seconds without violating jerk & acceleration limits.

### Demo

[![](http://img.youtube.com/vi/I3dN14Lfd8o/0.jpg)](http://www.youtube.com/watch?v=I3dN14Lfd8o "")





