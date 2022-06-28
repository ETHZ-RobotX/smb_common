# SMB_SLAM
This package contains a bunch of commodity scripts for running (simultaneous) localization & mapping.

For creating the map, [open3d_slam](https://github.com/leggedrobotics/open3d_slam) can be used.
The map generation is both possible in real-time on the system or from a rosbag, as well as by directly reading from the rosbag.
For the latter case, use the script `build_map_from_bag.launch` and specify the location of the rosbag.

An exemplary script with <path_to_launchfile> would be the following:
```bash
roslaunch smb_slam build_map_from_bag.launch rosbag_filepath:=<path_to_bagfile> launch_rviz:=true
```