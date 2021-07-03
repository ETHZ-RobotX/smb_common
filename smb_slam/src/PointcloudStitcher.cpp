/*
 * PointCloudStitcher.cpp
 *
 *  Created on: Aug 20, 2019
 *      Author: jelavice
 */

#include "smb_slam/PointcloudStitcher.hpp"

#include <cassert>
#include <iostream>
#include <fstream>
#include <memory>
#include <boost/filesystem.hpp>
#include <boost/assign/list_of.hpp>

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>

#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/serialization.h"
#include "pointmatcher_ros/deserialization.h"
#include "pointmatcher_ros/RosPointCloud2Deserializer.h"

#include <Eigen/StdVector>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#define DEBUG

namespace smb_slam {

namespace {
  using Point = pcl::PointXYZ;
  using Pointcloud = pcl::PointCloud<Point>;
}


const double xMin_ = -500.0;
const double xMax_ = 500.0;
const double yMin_ = -500.0;
const double yMax_ = 500.0;
const double zMin_ = -500.0;
const double zMax_ = 500.0;
const double removeInside_ = false;

PointCloudStitcher::PointCloudStitcher(ros::NodeHandle &nh)
    : nh_(nh)
{

}

PointCloudStitcher::~PointCloudStitcher(){
  std::cerr << "Saving pintcloud!" << std::endl;
  save();
}

void PointCloudStitcher::initialize()
{
  initFilters();
  initRos();
}

void PointCloudStitcher::initFilters()
{
  // load YAML config
  std::string path = ros::package::getPath("smb_slam") + std::string("/config/compslam/pointcloud_stitcher.yaml");
  std::ifstream ifs(path.c_str());
  if (!ifs.good()) {
    throw std::runtime_error("Cannot open config file");
  }
  pointCloudFilters_ = std::make_unique<PM::DataPointsFilters>(ifs);
}

void PointCloudStitcher::initRos()
{
  nh_.param<std::string>("stitching_frame", concatenatingFrame_, "BASE");

  nh_.param<std::string>("output_path", output_path_, ros::package::getPath("smb_slam") + "/maps/compslam_map.pcd");

  //subscriber
  std::string inputPointCloudTopic = "/velodyne_points_self_filtered";
  nh_.param<std::string>("input_cloud_topic", inputPointCloudTopic,
                         "/velodyne_points_self_filtered");
  inputCloudSubscriber_ = nh_.subscribe(inputPointCloudTopic, 5,
                                        &PointCloudStitcher::inputPointCloudCallback, this);
  std::cout << "Subscribing to: " << inputPointCloudTopic << std::endl;
  //publishers
  std::string stitchedPointCloudTopic="stitched_cloud";
  // nh_.param<std::string>("publishers/pcl_stitcher_stithced_cloud/topic", stitchedPointCloudTopic,
  //                        "pcl_stitcher/stitched_cloud");
  const bool isLatchPublisher = false;
  stitchedPointCloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>(stitchedPointCloudTopic, 1,
                                                                   isLatchPublisher);

  debugCloudPublisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/pcl_stitcher/debug_cloud", 1,
                                                                 isLatchPublisher);

  //advertise services
  std::string startAccumulatingPointsService= "start_stitching";
  // nh_.param<std::string>("servers/pcl_stitcher_start_accumulating/service",
  //                        startAccumulatingPointsService, "/pcl_stitcher/start_accumulating_point");
  accumulateLidarPointsService_ = nh_.advertiseService(
      startAccumulatingPointsService, &PointCloudStitcher::accumulateLidarPointServiceCallback,
      this);

  std::string publishAccumultedPointsService = "publish_stitched_cloud";
  // nh_.param<std::string>("servers/pcl_stitcher_publish_stitched/service",
  //                        publishAccumultedPointsService,
  //                        "/pcl_stitcher/publish_stitched_point_cloud");
  concatenatedCloudPublishingService_ = nh_.advertiseService(
      publishAccumultedPointsService, &PointCloudStitcher::publishStitchedPointCloudServiceCallback,
      this);

  // filteringToConcatenatigFrameListener_.waitForTransform(concatenatingFrame_, filteringFrame_,
  //                                                        ros::Time(0), ros::Duration(20.0));

    nh_.param<bool>("accumulate_on_startup",
                         isAccumulatePoints_, false);
    std::cout << "Accumulate on startup: " << isAccumulatePoints_ << std::endl;

    lastSaveTime_ = ros::Time::now();

}

bool PointCloudStitcher::accumulateLidarPointServiceCallback(std_srvs::Trigger::Request &req,
                                                             std_srvs::Trigger::Response &res)
{
  isAccumulatePoints_ = true;
  ROS_INFO_STREAM("Starting to accumulate points");
  return true;
}

bool PointCloudStitcher::publishStitchedPointCloudServiceCallback(std_srvs::Trigger::Request &req,
                                                                  std_srvs::Trigger::Response &res)
{
  //isAccumulatePoints_ = false;
  sensor_msgs::PointCloud2 outputPointCloud = pointmatcher_ros::pointMatcherCloudToRosMsg<float>(
      concatenatedCloud_.dataPoints_, concatenatingFrame_, lastInputPointCloudTimestamp_);

  ROS_INFO_STREAM(
      "Publishing accumulated points, number of them: " << concatenatedCloud_.dataPoints_.getNbPoints());

  stitchedPointCloudPub_.publish(outputPointCloud);

  save();

  return true;
}

void PointCloudStitcher::save() const{

  PM::TransformationParameters tfConcatenatingToOdomFrame = lookupTransform(
      filteringToConcatenatigFrameListener_, "odom", concatenatingFrame_);

  DP cloudOut;
  applyRigidBodyTransform(tfConcatenatingToOdomFrame, concatenatedCloud_.dataPoints_,
                          &cloudOut);


  std::cout << "SAving cloud with: " << concatenatedCloud_.dataPoints_.getNbPoints() << " points" << std::endl;
  //concatenatedCloud_.clear();
  const std::string path = output_path_;
  cloudOut.save(path);
  std::cout << "Saved to: " << path << std::endl;
  // sensor_msgs::PointCloud2 cloudOutRos = pointmatcher_ros::pointMatcherCloudToRosMsg<float>(
  //     cloudOut, "odom", lastInputPointCloudTimestamp_);
  // Pointcloud cloud;
  // pcl::fromROSMsg(cloudOutRos, cloud);
  // pcl::io::savePCDFile(path, cloud);

}

void PointCloudStitcher::inputPointCloudCallback(const sensor_msgs::PointCloud2& rosInputPointCloud)
{
  if (!isAccumulatePoints_) {
    return;
  }

  //save Time
  lastInputPointCloudTimestamp_ = rosInputPointCloud.header.stamp;

  //transform pointcloud to PM
  //DP inputPointCloud = pointmatcher_ros::rosMsgToPointMatcherCloud<float>(rosInputPointCloud);
  DP inputPointCloud = pointmatcher_ros::RosPointCloud2Deserializer<float>::deserialize(rosInputPointCloud);

  concatenatingFrame_ = rosInputPointCloud.header.frame_id;


//   //lookop transform
//   PM::TransformationParameters tfLidarToFilteringFrame = lookupTransform(
//       lidarToFilteringFrameListener_, filteringFrame_, rosInputPointCloud.header.frame_id);
//   //std::cout << "input cloud num features: " << inputPointCloud.features.rows() << std::endl;

//   //apply rigid body transformation
//   DP cloudInFilteringFrame;
//   applyRigidBodyTransform(tfLidarToFilteringFrame, inputPointCloud, &cloudInFilteringFrame);
//   //std::cout << "rotated input cloud num features: " << cloudInFilteringFrame.features.rows() << std::endl;
//   //stop if there are no points
//   if (cloudInFilteringFrame.getNbPoints() == 0) {
//     return;
//   }

//   //apply filters
//   cropPointsInPlace(&cloudInFilteringFrame);
// //  std::cout  << "Points cropped, size of cloud in the filtering frame: " << cloudInFilteringFrame.getNbPoints() << std::endl;
// //  std::cout << "cropped input cloud num features: " << cloudInFilteringFrame.features.rows() << std::endl;

//   PM::TransformationParameters tfFilteringToConcatenatingFrame = lookupTransform(
//       filteringToConcatenatigFrameListener_, concatenatingFrame_, filteringFrame_);

//   DP cloudInConcatenatingFrame;
//   applyRigidBodyTransform(tfFilteringToConcatenatingFrame, cloudInFilteringFrame,
//                           &cloudInConcatenatingFrame);

//  std::cout  << "Points rotated again, size of cloud in the concatenating frame: " << cloudInConcatenatingFrame.getNbPoints() << std::endl;
//  std::cout << "cropped input cloud num features: " << cloudInConcatenatingFrame.features.rows() << std::endl;

  concatenatePointClouds(inputPointCloud);

  if (concatenatedCloud_.getSize() < 50) {
    ROS_WARN_STREAM_THROTTLE(2.0,"Concatenated cloud has not enough points, skipping the filtering step");
  } else {
    try {
      pointCloudFilters_->apply(concatenatedCloud_.dataPoints_);
    } catch (const std::exception &e) {
      ROS_ERROR_STREAM_THROTTLE(2.0,"Applying filter threw an exception: " << e.what());
    }

  }

#ifdef DEBUG
  publishDebugCloud();
#endif
}

PointCloudStitcher::PM::TransformationParameters PointCloudStitcher::lookupTransform(
    const tf::TransformListener &transformListener, const std::string &targetFrame,
    const std::string &sourceFrame) const
{
  //lookop transform
  PM::TransformationParameters transform;
  try {
    transform = pointmatcher_ros::transformListenerToEigenMatrix<float>(
        transformListener, targetFrame, sourceFrame, lastInputPointCloudTimestamp_);

  } catch (tf::TransformException &ex) {
    ROS_ERROR_THROTTLE(2.0,"PCL_STITCHER: Failure looking up the transform %s\n", ex.what());
  }

  return transform;
}

void PointCloudStitcher::applyRigidBodyTransform(PM::TransformationParameters &transform,
                                                 const DP &inputCloud, DP *transformedCloud) const
{
  //apply rigid body transformation
  std::shared_ptr<PM::Transformation> rigidBodyTransformation = PM::get().TransformationRegistrar
      .create("RigidTransformation");

  if (inputCloud.getNbPoints() == 0) {
    ROS_WARN_STREAM_THROTTLE(2.0," No points in the input cloud. Cannot apply rigid body transform!!");
    return;
  }

  try {
    *transformedCloud = rigidBodyTransformation->compute(inputCloud, transform);
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM_THROTTLE(2.0,"Ri gid body transform threw an exception: " << e.what());
  }

}

std::shared_ptr<PointCloudStitcher::PM::DataPointsFilter> PointCloudStitcher::createFilter() const
{
  std::shared_ptr<PM::DataPointsFilter> boundingBoxFilter = PM::get().DataPointsFilterRegistrar
      .create(
      "BoundingBoxDataPointsFilter",
      boost::assign::map_list_of("xMin", PointMatcherSupport::toParam(xMin_))(
          "xMax", PointMatcherSupport::toParam(xMax_))("yMin", PointMatcherSupport::toParam(yMin_))(
          "yMax", PointMatcherSupport::toParam(yMax_))("zMin", PointMatcherSupport::toParam(zMin_))(
          "zMax", PointMatcherSupport::toParam(zMax_))(
          "removeInside", PointMatcherSupport::toParam(removeInside_)));
  return boundingBoxFilter;
}

void PointCloudStitcher::cropPointsInPlace(DP *inputCloud) const
{
  if (inputCloud->getNbPoints() == 0) {
    ROS_WARN_STREAM_THROTTLE(2.0," No points in the input cloud. Cannot crop in place!");
    return;
  }
  auto croppingPointCloudFilter = createFilter();

  try {
    croppingPointCloudFilter->inPlaceFilter(*inputCloud);
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM_THROTTLE(2.0,"Inplace filter threw an exception: " << e.what());
  }

}

void PointCloudStitcher::concatenatePointClouds(const DP &newCloud)
{

  if (newCloud.getNbPoints() == 0){
    return;
  }

  if (concatenatedCloud_.getSize() == 0) {
    concatenatedCloud_.dataPoints_ = newCloud;
    concatenatedCloud_.header_.stamp = lastInputPointCloudTimestamp_;
    concatenatedCloud_.header_.frame_id = concatenatingFrame_;
  } else {
    try {
      concatenatedCloud_.dataPoints_.concatenate(newCloud);
    } catch (const std::exception &e) {
      ROS_ERROR_STREAM_THROTTLE(2.0,"Concatenating threw an exception: " << e.what());
    }
  }
}

void PointCloudStitcher::publishDebugCloud() const
{

  sensor_msgs::PointCloud2 outputPointCloud = pointmatcher_ros::pointMatcherCloudToRosMsg<float>(
      concatenatedCloud_.dataPoints_, concatenatingFrame_, lastInputPointCloudTimestamp_);

  debugCloudPublisher_.publish(outputPointCloud);

}

} /* namespace smb_slam */

