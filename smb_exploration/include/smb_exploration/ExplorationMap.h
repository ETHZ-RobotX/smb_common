#pragma once
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


namespace smb_exploration {

class ExplorationMap
{
private:
    // nodehandle
    ros::NodeHandle nh_;
    
    // subsriber
    ros::Subscriber scan_sub_;

    // publisher
    ros::Publisher occ_pub_;

    // costmap
    costmap_2d::Costmap2DROS* costmap2d_ros_;

    // tf
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;  

    // callbacks
    void scanCallback(const sensor_msgs::LaserScanConstPtr& scan);
    
public:
    ExplorationMap(ros::NodeHandle nh);
};

}