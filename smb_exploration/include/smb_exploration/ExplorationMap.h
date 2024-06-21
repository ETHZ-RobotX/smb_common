#pragma once
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <laser_geometry/laser_geometry.h>


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
    ros::Publisher pcd_pub_;

    // costmap
    costmap_2d::Costmap2DROS* costmap2d_ros_;

    // tf
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;  

    laser_geometry::LaserProjection projector_;  // Used to project laser scans into point clouds

    // frames
    std::string global_frame_;

    // map params
    int min_x_;
    int min_y_;
    int max_x_;
    int max_y_;
    int update_block_size_{10};
    double resolution_;

    // callbacks
    void scanCallback(const sensor_msgs::LaserScanConstPtr& scan);

    bool shouldUpdateBounds(const float min_x,const float min_y, const float max_x, const float max_y);

    void updateBounds(const float min_x,const float min_y, const float max_x, const float max_y);

public:
    ExplorationMap(ros::NodeHandle nh);
};

}