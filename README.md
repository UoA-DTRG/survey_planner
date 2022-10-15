# Survey Planner
This repository contains code for trajectory planning within complex, 3D spaces

## Prerequisites
While this repository should remain compatible with other versions of ROS1/PX4, it has been developed and tested with the following:

 - Ubuntu 20.04.3
 - ROS1 Noetic
 - PX4 SITL v1.12.3

This planner has been specifically designed to work with our trajectory tracker (Survey Tracker)[]. If you haven't done so already, setup Survey Tracker first in your catkin workspace.

## Installation
Since Survey Planner depends heavily on Survey Tracker, we will assume that you've already set up our trajectory tracker before continuing.

First, install dependencies
```
sudo apt install libeigen3-dev
```

Follow the installation instructions [here](http://ceres-solver.org/installation.html) to install Google Ceres. Note that you will need to install Ceres **1.14.0**.

Finally, build this package in the catkin workspace you've cloned it to
```
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build 
```

## Running the package
Once you have the simulator and trajectory tracker running, simply run

```
source devel/setup.bash && roslaunch path_search uav_sim.launch
```

## Acknowledgements
If you use this work in your academic work, please cite at least one of our papers
```
[1] Lin, T. and Stol, K.A., Autonomous Surveying of Plantation Forests using Multi-Rotor UAVs, Drones
[2] Lin, T. and Stol, K.A., Faster Navigation of Semi-Structured Forest Environmentsusing Multi-Rotor UAVs, Robotica
[3] Lin, T. and Stol, K.A., Fast Trajectory Tracking of Multi-Rotor UAVs using First-Order Model Predictive Control, 2021 Australian Conference on Robotics and Automation (ACRA), Melbourne, Australia
```