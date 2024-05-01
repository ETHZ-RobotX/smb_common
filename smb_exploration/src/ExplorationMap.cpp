#include "smb_exploration/ExplorationMap.h"

namespace smb_exploration {

ExplorationMap::ExplorationMap(ros::NodeHandle nh) 
    : nh_(nh), tfListener_(tfBuffer_)
{
    // topic names
    ROS_INFO_STREAM("Creating exploration map ... ...");
    std::string scan_topic = "/scan";
    std::string occ_topic = "/map";

    // 
    costmap2d_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tfBuffer_);
    // subscriber
    scan_sub_ = nh_.subscribe(scan_topic, 1, &ExplorationMap::scanCallback, this);

    // publisher
    occ_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(occ_topic, 1, false);
    ROS_INFO_STREAM("Created exploration map.");

}

void ExplorationMap::scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
    
}

} //namespace smb_exploration