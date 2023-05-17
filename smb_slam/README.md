# SMB_SLAM
This package contains a bunch of commodity scripts for running (simultaneous) localization & mapping.

For creating the map, [open3d_slam](https://github.com/leggedrobotics/open3d_slam) can be used. We have created a new branch for the robotics summer school. Make sure that you are using the correct branch ([robotics_summer_school_2023](https://github.com/leggedrobotics/open3d_slam/tree/robotics_summer_school_2023)).

It is necessary to follow the instructions [here](https://open3d-slam.readthedocs.io/en/latest/installation.html) to install all dependencies before compile `smb_slam`.

You can then proceed to build the `smb_slam` package using the following command from the worksapce: `catkin build smb_slam`

### Running Open3d SLAM

The map generation is both possible in real-time on the system or from a rosbag, as well as by directly reading from the rosbag. 

Here are different modes of operation:

1. Building a Map Offline (Using Rosbag):

     Run the following command to start running online SLAM. You can save the map at any point by using rosservice or the map will automatically get saved upon closing the node. Make sure that you specify the correct full path of the bag file using the `rosbag_full_path` argument. Alternatively, you can instead specify the folder path and filename using the arguments `bag_folder_path` and `bag_filename` repectively.
     
      ```
      roslaunch smb_slam build_map_from_bag.launch rosbag_full_path:="ABSOULTE PATH TO BAG FILE"
      ```

2. Running in Online SLAM Mode:

      Make sure that `smb smb.launch` is running correctly. Now simply run the following command to start running online SLAM. You can save the map at any point by using rosservice or the map will automatically get saved upon closing the node.
      
      ```
      roslaunch smb_slam online_slam.launch
      ```

3. Localization Mode:

      If you have already created a map of the environment and would like to localize the robot in this prebuilt map, use the following command. Make sure that you correctly specify the path of the pcd map (Default path is `data/maps/map.pcd`). We recommend using this for localization instead of the ICP Localization package. 
      ```
      roslaunch smb_slam localization_open3dslam.launch
      ```
     Initial estimate for the robot in the map will need to be provided. This can be done by using the 2D pose estimate in RVIZ.

### Map Saving

The map can be saved at any point by using rosservice from a new terminal. Run the following command from a new sourced terminal:
```
rosservice call /mapping_node/save_map
```
Alternatively there is an option for autosaving upon exit which can be switched on from the param files (`params.saving.save_at_mission_end = true`). This is switched off by default.
