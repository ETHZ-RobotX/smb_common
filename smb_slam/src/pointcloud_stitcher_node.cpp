// pointmatcher


// ros
#include <ros/ros.h>
#include "smb_slam/PointcloudStitcher.hpp"




int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pointcloud_stitcher_node");
  ros::NodeHandle nh("~");

  smb_slam::PointCloudStitcher pclStitcher(nh);

  pclStitcher.initialize();

  ros::spin();

  return 0;
}
