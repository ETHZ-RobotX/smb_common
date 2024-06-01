#include <ros/ros.h>

#include "smb_exploration/PointCloudTransformer.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "pointcloud_transformer_node");
  ros::NodeHandle nh_private("~");

  smb_exploration::PointCloudTransformer pointcloud_transformer(nh_private);

  ros::AsyncSpinner spinner(2);  // Use 2 threads
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
