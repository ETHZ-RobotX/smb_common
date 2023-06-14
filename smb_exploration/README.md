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

[GMapping](http://wiki.ros.org/gmapping) is used to create a 2D Occupancy map of the environment which is then used by the exploration planner to generate a goal point based on frontiers. You can have a look into the parameters [here](http://wiki.ros.org/explore_lite). Some tuning might also be required for GMapping. The goal provided by the exploration algorithm is then reached using movebase.

### Run Instructions 

The instructions are pretty much the same for running the planner in simulation or on the real-robot. Once you have the simulation (or real robot smb.launch) running, from a separate sourced terminal run the following command to launch the planner (Note: you will have had to build smb_navigation package before as well). Also make sure that you provide the correct global frame:
```
roslaunch smb_navigation navigate2d_ompl.launch global_frame:=tracking_camera_odom
```
Now, in a new sourced terminal, run the following command to launch GMapping and the frontier-based exploration planner
```
roslaunch smb_exploration smb_rss_frontier.launch
```
You can visualize the 2D Occupancy map on the ```/map``` topic and the frontiers on the ```/explore/frontiers``` in RViz.


