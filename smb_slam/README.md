# SMB_SLAM
This package contains a bunch of commodity scripts for running (simultaneous) localization & mapping using [open3d_slam](https://github.com/ETHZ-RobotX/open3d_slam_advanced_rss_2024_public).

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

1. Running in Online SLAM Mode:

      Make sure that `smb smb.launch` is running correctly. Now simply run the following command to start running online SLAM. You can save the map at any point by using rosservice or the map will automatically get saved upon closing the node. Advanced features such as `loop closure`, `space carving` and `submap-to-submap` registration are disabled by default as SMBs are computationally already loaded. Refer to the parameter file `/smb_slam/config/open3d_slam/param_robosense_rs16.lua` and the below table for parameter details.
      
      ```
      roslaunch smb_slam online_slam.launch
      ```

2. Localization Mode:

      If you have already created a map of the environment and would like to localize the robot in this prebuilt map, use the following command. Make sure that you correctly specify the path of the pcd map (Default path is `data/maps/map.pcd`).
      ```
      roslaunch smb_slam localization.launch 
      ```
     Initial estimate for the robot in the map will need to be provided. This can be done by using the 2D pose estimate in RVIZ.
   
For all the above cases, if you run in simulation, you can launch rviz by setting the `launch_rviz:=true` to the launch command.


3. Building a Map Offline (Replay Mode):

     Run the following command to start running a replayer node that uses an external odometry specified in the launch file and the point cloud. Different than the online SLAM pipeline, this replayer runs sequentially. This means, the node will not skip measurements and instead it will wait for computation of the previous point cloud to finish before moving on. As a result, in compute limited systems might run slower than real-time and vice-versa, i.e. faster than real-time in powerful devices. This node saves the generated map, a rosbag that contains the registered point clouds as well as the calculated poses and finally, the path of the robot as a .pcd file such that the user can visualize the path next to the map. The saving folder is specified within the `.launch` file.
     ```
     roslaunch smb_slam replay_SLAM.launch
     ```

> **Note**: The user is responsible to make sure that the specified topics are in the bag. The node will guide if there is something missing.


### Map Saving

The map can be saved at any point by using rosservice from a new terminal. Run the following command from a new sourced terminal:
```
rosservice call /mapping/save_map
```
Alternatively there is an option for autosaving upon exit which can be switched on from the param files (`params.saving.save_map = true`).


## Important Parameters

Other than the parameters exposed to the ROS parameter server (the parameters user can set with `.launch` files) there are few important parameters located in `/smb_slam/config/open3d_slam/param_robosense_rs16.lua` file or similar files to this.

| Lua Parameter                             | Type     | Description                                                                                                                                                                                                        |
| ----------------------------------------- | -------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `params.odometry.use_odometry_topic_instead_of_scan_to_scan` | `bool` | If true, O3D uses the external odometry specified in the `.launch` file. |
| `params.mapper_localizer.is_use_map_initialization` | `bool`  | If true, O3D runs in localization mode and uses the given map, specified in the LUA file. |
| `params.mapper_localizer.is_attempt_loop_closure ` | ` bool` | if true, O3D does loop-closures. This feature is power hungry, use responsibly. |
| `params.mapper_localizer.is_carving_enabled` | `bool` | if true, O3D does space carving. If a previously occupied space is re-observed to be empty, that voxel is cleaned. Effective against dynamic obstables. Increses computation is enabled.|
| `params.submap.submap_size` | `double` | The submap size in meters, if increased each submap contains more data but computation increases.|
| `params.map_builder.map_voxel_size ` | `double` | The voxel size of the map, subsequently of the submap. Defines the resolution of the map that is used for localization purposes. |
| `params.mapper_localizer.scan_to_map_registration.icp.reference_cloud_seting_period` | `double` | The refence setting period in seconds. If you decrease (> 0.0s) the localization becomes more robust but computation increases. If rapid motions are done, the decrease in the period can help.|
| `params.mapper_localizer.scan_to_map_registration.scan_processing.voxel_size` | `double` | The voxel size used for the registration. This parameter should not be set smaller than the map voxel size, as it would be meaningless. If set smaller the localization resolution increases but also sensitivity to noise and computational limitations. |


> **Note**: There are multiple advanced parameters in Open3D, to not complicated simple operation these are not detailed here. Follow the naming of the variables to infer usage.
