#include <ros/ros.h>

#include "smb_exploration/ExplorationMap.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "exploration_map_node");
  ros::NodeHandle nh_private("~");

  smb_exploration::ExplorationMap exploration_map(nh_private);

  ros::AsyncSpinner spinner(2);  // Use 2 threads
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
