# SMB_SLAM
This package contains a bunch of commodity scripts for running (simultaneous) localization & mapping using [open3d_slam](https://github.com/ETHZ-RobotX/open3d_slam_advanced_rss_2024).

Follow instructions the instructions below to build ```smb_slam```.

1. Make sure that you have installed the repositories following the instructions [here](https://ethz-robotx.github.io/SuperMegaBot/core-software/installation_core.html) for core SMB software.
2. Install dependencies for Open3d_slam for which you can follow these steps:
     ```bash
     sudo apt install libgoogle-glog-dev libglfw3 libglfw3-dev ros-noetic-jsk-rviz-plugins liblua5.2-dev
     sudo add-apt-repository ppa:roehling/open3d
     sudo apt update
     sudo apt install libopen3d-dev cmake
     ```
3. You can then proceed to build the `smb_slam` package using the following command from the worksapce: `catkin build smb_slam`

### Running Open3d SLAM

The map generation is both possible in real-time on the system or from a rosbag, as well as by directly reading from the rosbag. 

Here are different modes of operation:

1. Building a Map Offline (Using Rosbag):

     Run the following command to start running online SLAM. You can save the map at any point by using rosservice or the map will automatically get saved upon closing the node. Make sure that you specify the correct full path of the bag file using the `rosbag_full_path` argument. Alternatively, you can instead specify the folder path and filename using the arguments `bag_folder_path` and `bag_filename` repectively. The `loop closure` will be enabled by default in this case of offline map creation.
     
      ```
      roslaunch smb_slam build_map_from_bag.launch rosbag_full_path:="ABSOULTE PATH TO BAG FILE"
      ```

2. Running in Online SLAM Mode:

      Make sure that `smb smb.launch` is running correctly. Now simply run the following command to start running online SLAM. You can save the map at any point by using rosservice or the map will automatically get saved upon closing the node. The `loop closure` is switched off by default for this operation and can be turned on using ```loop_closure:=true``` argument (Though we don't recommend it due to computational limits).
      
      ```
      roslaunch smb_slam online_slam.launch
      ```

3. Localization Mode:

      If you have already created a map of the environment and would like to localize the robot in this prebuilt map, use the following command. Make sure that you correctly specify the path of the pcd map (Default path is `data/maps/map.pcd`).
      ```
      roslaunch smb_slam localization.launch 
      ```
     Initial estimate for the robot in the map will need to be provided. This can be done by using the 2D pose estimate in RVIZ.
   
For all the above cases, if you run in simulation, you can launch rviz by setting the `launch_rviz:=true` to the launch command.

### Map Saving

The map can be saved at any point by using rosservice from a new terminal. Run the following command from a new sourced terminal:
```
rosservice call /mapping_node/save_map
```
Alternatively there is an option for autosaving upon exit which can be switched on from the param files (`params.saving.save_at_mission_end = true`). This is switched off by default.
