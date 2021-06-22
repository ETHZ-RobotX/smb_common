#include <odometry_conversion/OdometryConversion.hpp>

using namespace odometry_conversion;

int main(int argc, char** argv) {
  ros::init(argc, argv, "odometry_conversion");
  ros::NodeHandle nodeHandle("~");
  OdometryConversion odometryConversion(nodeHandle);
  ros::spin();
  return 0;
}
