# smb_msf_graph

Package to run [Graph-MSF](https://github.com/leggedrobotics/graph_msf/) on the SuperMegaBot.

## Compile gtsam_catkin

To compile the required packages, simply run
```bash
catkin build gtsam_catkin
```
This will build gtsam in you workspace and could take up to 5 minutes depending on your CPU.

**NOTE:** It will only take that long the first time you build `gtsam_catkin` in this workspace. Even after cleaning your build with `catkin clean`, it will be much faster, as the compiled binaries are stored inside the `lib/gtsam_catkin` repository.

## Build smb_msf_graph
To build `smb_msf_graph` run
```bash
catkin build smb_msf_graph
```
This will compile all dependencies present in the workspace, including [open3d_slam](https://github.com/ETHZ-RobotX/open3d_slam_advanced_rss_2024).

# Run smb_msf_graph
## Run msf_graph with online SLAM
This will automatically start open3d_slam in online SLAM mode
```bash
roslaunch smb_msf_graph smb_msf_graph.launch
```
## Run msf_graph in localization mode
In the first terminal launch open3d_slam in localization mode
```bash
roslaunch smb_slam localization.launch
```
**Note:** please refer to the [README](https://github.com/ETHZ-RobotX/smb_common/blob/devel/smb_slam/README.md) file in smb_slam

After you are localized run in a second terminal
```bash
roslaunch smb_msf_graph smb_msf_graph.launch launch_o3d_slam:=false
```
