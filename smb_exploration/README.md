# SMB_EXPLORATION

This package provides launch files for 2 separate exploration planners namely [gbplanner](https://github.com/ETHZ-RobotX/gbplanner_ros) and a frontier-based planner [exploration_lite](http://wiki.ros.org/explore_lite).

## Build instructions

Run the following command to build both the exploration planners and its required dependencies

```
catkin build smb_exploration
```

## GBPlanner

This planner makes use of Voxblox to create a 3D Occupancy map after which the exploration waypoints are calculated using information gain by raycasting. The planner planner then provides a trajectory which is tracked by using a simple path tracking algorithm such as pure pursuit. Checkout the [official documentation](https://github.com/ntnu-arl/gbplanner_ros/wiki) to know more about the planner and the different params. 

### Run Instructions 

The instructions are pretty much the same for running the planner in simulation or on the real-robot. Once you have the simulation (or real robot smb.launch) running, from a separate sourced terminal run the following command:

```
roslaunch smb_exploration smb_rss_gbplanner.launch
```

Note, if you are running it on the real-robot, you will need to run the rviz visualization seperately on your computer. Make sure to use the correct rviz config file (provided in th smb_exploration/config/gbplanner/rviz) so that you have access to the interface.
You will need to carry out an initialization maneuver before starting the planner to clear out the empty space surrounding the robot.


## Frontier-based Exploration Planner

