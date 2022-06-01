#include <ros/ros.h>
#include <smb_mpc/SmbController.hpp>

using namespace smb_mpc;

int main(int argc, char** argv){
    ros::init(argc, argv, "smb_mpc_node");
    ros::NodeHandle nh;
    SmbController controller(nh);
    ros::spin();
}