#pragma once
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Trigger.h>

// pointmatcher

#include <pointmatcher_ros/StampedPointCloud.h>
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/IO.h"

#include <memory>

namespace smb_slam {


class PointCloudStitcher {

  using PM = PointMatcher<float>;
  using DP = PM::DataPoints;
  using IO = PointMatcherIO<float>;


 public:

  PointCloudStitcher() = delete;
  explicit PointCloudStitcher(ros::NodeHandle &nh);
  virtual ~PointCloudStitcher();

  void initialize();

 private:

  void inputPointCloudCallback(const sensor_msgs::PointCloud2& rosInputPointCloud);
  bool accumulateLidarPointServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool publishStitchedPointCloudServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  void initRos();
  void initFilters();
  PM::TransformationParameters lookupTransform(const tf::TransformListener &TransformListener, const std::string &sourceFrame, const std::string &targetFrame) const;
  void applyRigidBodyTransform(PM::TransformationParameters &transform, const DP &inputCloud, DP *transformedCloud) const;
  std::shared_ptr<PM::DataPointsFilter> createFilter() const;
  void cropPointsInPlace(DP *inputCloud) const;
  void concatenatePointClouds(const DP &newCloud);
  void save() const;

  void publishDebugCloud() const;

  std::unique_ptr<PM::DataPointsFilters> pointCloudFilters_;
  tf::TransformListener lidarToFilteringFrameListener_;
  tf::TransformListener filteringToConcatenatigFrameListener_;
  ros::Publisher stitchedPointCloudPub_;
  ros::Publisher debugCloudPublisher_;
  pointmatcher_ros::StampedPointCloud concatenatedCloud_;

  ros::Subscriber inputCloudSubscriber_;

  ros::ServiceServer concatenatedCloudPublishingService_;
  ros::ServiceServer accumulateLidarPointsService_;
  bool isAccumulatePoints_ = false;


  ros::Time lastInputPointCloudTimestamp_;

  ros::NodeHandle &nh_;
  std::string concatenatingFrame_ = "odom";
  std::string filteringFrame_ = "base_link";
  std::string output_path_ = "";
  ros::Time lastSaveTime_;
  double saveEveryNSec_= 5.0;

};



} /* namespace smb_slam */