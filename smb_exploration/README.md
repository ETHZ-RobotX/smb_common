# SMB_EXPLORATION

This package provides launch files for 2 separate exploration planners namely [gbplanner](https://github.com/ETHZ-RobotX/gbplanner_ros) and a frontier-based planner [exploration_lite](http://wiki.ros.org/explore_lite).

## Build instructions

Run the following command to build both the exploration planners and their required dependencies

```
catkin build smb_exploration
```

## Frontier-based Exploration Planner

[GMapping](http://wiki.ros.org/gmapping) is used to create a 2D Occupancy map of the environment which is then used by the exploration planner to generate a goal point based on frontiers. You can have a look into the parameters [here](http://wiki.ros.org/explore_lite). Some tuning might be required for GMapping. The goal provided by the exploration algorithm is then reached using movebase.

### Run Instructions 

The instructions are pretty much the same for running the planner in simulation or on the real-robot. Once you have the simulation (or real robot smb.launch) running, from a separate sourced terminal run the following command to launch the planner (Note: you will have had to build ```smb_navigation``` package before as well). 

#### Option 1: Running with tracking camera odometry
You will require three sourced terminals for this. In each of the new terminals run the following commands.
```bash
catkin build smb_exploration smb_navigation # Building the necessary packages if not done before
```
```bash
roslaunch smb smb.launch launch_tracking_cam:=true # Use the Gazebo equivalent if running in simulation with tracking cam flag
roslaunch smb_exploration smb_rss_frontier.launch map_frame:=gmap_map publish_odom_to_map:=true # Exploration + GMapping
roslaunch smb_navigation navigate2d_ompl.launch global_frame:=tracking_camera_odom exploration:=true global_frame:=gmap_map # Path Planner
```
You can visualize the 2D Occupancy map on the ```/map``` topic and the frontiers on the ```/explore/frontiers``` in RViz.

#### Option 2: Running with Online SLAM + MSF Graph
You will require four sourced terminals for this. In each of the new terminals run the following commands. Make sure that you are consistent with the odometry topics and frame names. Additionally, you will also have had the ```smb_msf_graph``` built.
```bash
catkin build smb_exploration smb_navigation smb_msf_graph # Building the necessary packages if not done before
```
```bash
roslaunch smb smb.launch # Use the Gazebo equivalent if running in simulation
roslaunch smb_msf_graph smb_msf_graph.launch # To launch the msf Graph State Estimation
roslaunch smb_exploration smb_rss_frontier.launch odom_frame:=odom_graph_msf map_frame:=gmap_map publish_odom_to_map:=true # Exploration + GMapping
roslaunch smb_navigation navigate2d_ompl.launch exploration:=true global_frame:=gmap_map odom_topic:=/graph_msf/est_odometry_odom_imu # Path Planner
```


## GBPlanner

This planner makes use of Voxblox to create a 3D Occupancy map after which the exploration waypoints are calculated using information gain by raycasting. The planner then provides a trajectory which is tracked by using a simple path-tracking algorithm such as pure pursuit. Check out the [official documentation](https://github.com/ntnu-arl/gbplanner_ros/wiki) to know more about the planner and the different params.

Note: This planner has not been tested extensively on the robot so you might need to tune quite some params as well as fix some issues.

### Run Instructions 

The instructions are pretty much the same for running the planner in simulation or on the real-robot. Once you have the simulation (or real robot smb.launch) running, from a separate sourced terminal run the following command:

```
roslaunch smb_exploration smb_rss_gbplanner.launch
```

Note, if you are running it on the real-robot, you will need to run the rviz visualization separately on your computer. Make sure to use the correct rviz config file (provided in the smb_exploration/config/gbplanner/rviz) so that you have access to the interface.
You will need to carry out an initialization maneuver before starting the planner to clear out the empty space surrounding the robot.
