# README

## TODO

- [ ] script to extract artifact from /W_artifact to csv
- [ ] test for simple waypoint list
- [ ] test for overall waypoint list
- [ ] to be updated
- [ ] (opt) use vio in msf

## Connection

```sh
#Wifi (SSID): SMB_264
#Password: SMB_264_RSS

ssh team7@10.0.4.5
#Password: smb
```

- start the robot

  ```sh
  roslaunch smb smb.launch command_smb:=True
  roslaunch smb_navigation navigate2d_ompl.launch global_frame:=tracking_camera_odom robot_base_frame:=base_link
  ```

- inspect from PC

  ```sh
  roslaunch smb_opc opc.launch
  ```

## On-site exploration

- start navigation

  :construction:

- rosbag record

  :construction:

  > I think we may direct rename with `-O NAME`, `--output-name=NAME`

  ```sh
  rosbag record -O ~/final_test.bag /versavis/cam0/image_raw /sensor_tf_frame /odom <need add topics after>
  ```

  - required topic for detection

    > https://github.com/ethz-asl/artefact_mapping

    ```
    /versavis/cam0/image_raw
    /blackfly_right_optical_link
    /odom
    ```

  - required topic for building new map

    ```
    
    ```

## Post-processing

> assuming the rosbag_name as `final_test.bag`

## Download bag from SMB

```sh
scp team7@10.0.4.5:~/final_test.bag ~/Downloads/final_test.bag
```

- then we could share the bag with Yujie´s harddrive

### Building the map

- run catographer

- ```
  # 1st terminal
  roslaunch smb_slam cartographer_rviz.launch
  # 2nd ternimal
  roslaunch smb_slam mapping_offline_ct.launch bag_filename:=/<path>/<to>/<rosbag>/first_mission_wangen.bag
  # 3rd ternimal save the map as .pcd
  roslaunch smb_slam assets_writer_ct.launch bag_filenames:=/<path>/<to>/<rosbag>/first_mission_wangen.bag pose_graph_filename:=/<path>/<to>/<rosbag>/first_mission_wangen.bag.pbstream map_name:=wangen_map
  ```

  

### Reporting the artifact class and position

```
# 1st terminal
roscore
# 2nd ternimal
rosparam set use_sim_time true        # for simulation
roslaunch smb_slam localization.launch map_name:=garage_decimated.pcd launch_rviz:=true

# 3rd ternimal
rosbag play --clock artefacts_garage.bag

# 4th ternimal
# source different ws
source ~/artefact_mapping_ws/devel/setup.bash
roslaunch artefact_mapping artefact_mapping.launch


rostopic echo /W_landmark
rostopic echo /W_artefact
```

- remark
  - you need to set "darknet_classes" params in artefact_mapping.launch to detect different objects
  - https://github.com/ethz-asl/darknet_catkin/blob/master/data/coco.names