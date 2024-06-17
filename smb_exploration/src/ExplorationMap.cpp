#include "smb_exploration/ExplorationMap.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <costmap_2d/costmap_layer.h>


namespace smb_exploration {

ExplorationMap::ExplorationMap(ros::NodeHandle nh) 
    : nh_(nh), tfListener_(tfBuffer_)
{
    // topic names
    ROS_INFO_STREAM("Creating exploration map ... ...");
    std::string scan_topic;
    if(!nh_.getParam("global_costmap/scan_topic", scan_topic)){
      ROS_ERROR("Could not parse the parameter global_costmap/scan_topic");
      return;
    }

    std::string occ_topic;
    if(!nh_.getParam("global_costmap/occ_topic", occ_topic)){
      ROS_ERROR("Could not parse the parameter global_costmap/occ_topic");
      return;
    }
    
    std::string pcd_topic = "/explore_pcd";
    if(!nh_.getParam("global_costmap/global_frame", global_frame_)){
      ROS_ERROR("Could not parse the parameter global_costmap/global_frame");
      return;
    }
    ROS_INFO_STREAM("global_frame_ is " << global_frame_);

    costmap2d_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tfBuffer_);
    
    // get consistent resolution
    resolution_ = costmap2d_ros_->getLayeredCostmap()->getCostmap()->getResolution();

    // get consistent initial size
    min_x_ = costmap2d_ros_->getLayeredCostmap()->getCostmap()->getOriginX();
    min_y_ = costmap2d_ros_->getLayeredCostmap()->getCostmap()->getOriginY();
    max_x_ = min_x_ + costmap2d_ros_->getLayeredCostmap()->getCostmap()->getSizeInCellsX()*resolution_;
    max_y_ = min_y_ + costmap2d_ros_->getLayeredCostmap()->getCostmap()->getSizeInCellsY()*resolution_;

    // subscriber
    scan_sub_ = nh_.subscribe(scan_topic, 1, &ExplorationMap::scanCallback, this);

    // publisher
    occ_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(occ_topic, 1, false);
    pcd_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pcd_topic, 1, true);
    ROS_INFO_STREAM("Created exploration map.");

}

void ExplorationMap::scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
 
    // project the laser into a point cloud
    sensor_msgs::PointCloud2 cloud;
    cloud.header = scan->header;
    try
    {
        projector_.transformLaserScanToPointCloud(global_frame_, *scan, cloud, tfBuffer_);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("transform LaserScan to PointCloud failed Ignore this laser scan. what(): %s", ex.what());
        return; //ignore this message
    }

    // get the bounds of the pointcloud
    // Convert to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud, *cloud_pcl);
    pcd_pub_.publish(cloud);

    // Initialize min and max values
    float min_x = std::numeric_limits<float>::max(), max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max(), max_y = std::numeric_limits<float>::lowest();

    // Iterate over each point in the cloud
    for (const auto& point : cloud_pcl->points) {
        if (point.x < min_x) min_x = point.x;
        if (point.x > max_x) max_x = point.x;
        if (point.y < min_y) min_y = point.y;
        if (point.y > max_y) max_y = point.y;
    }

    // update the costmap bounds
    if(!shouldUpdateBounds(min_x, min_y, max_x, max_y)) return;

    updateBounds(min_x, min_y, max_x, max_y);
    // costmap2d_ros_->getLayeredCostmap()->resizeMap(int(max_x-min_x)*20, int(max_y-min_y)*20, 0.05, -5, -5);

    // std::vector<boost::shared_ptr<costmap_2d::Layer>>* plugins = costmap2d_ros_->getLayeredCostmap()->getPlugins();
    // for(auto plugin : *plugins)
    // {
    //     dynamic_cast<costmap_2d::CostmapLayer*>(plugin.get())->addExtraBounds(min_x, min_y, max_x, max_y);
    //     ROS_INFO_STREAM("update costmap bounds : " << min_x << " " << min_y << " " << max_x << " " << max_y);
    // }

}

bool ExplorationMap::shouldUpdateBounds(const float min_x, const float min_y, const float max_x, const float max_y)
{
    return (min_x < min_x_) || (min_y < min_y_) || (max_x > max_x_) || (max_y > max_y_);
}

void ExplorationMap::updateBounds(const float min_x, const float min_y, const float max_x, const float max_y)
{
    // // claim the mutex for thread safety
    // boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap2d_ros_->getLayeredCostmap()->getMutex()));

    // calculate the new bounds
    int new_min_x = std::min(int(std::floor(min_x / update_block_size_)*update_block_size_), min_x_);
    int new_min_y = std::min(int(std::floor(min_y / update_block_size_)*update_block_size_), min_y_);
    int new_max_x = std::max(int(std::ceil(max_x / update_block_size_)*update_block_size_), max_x_);
    int new_max_y = std::max(int(std::ceil(max_y / update_block_size_)*update_block_size_), max_y_);

    // TODO: cache the old map (the old obstacle layer)
    std::vector<boost::shared_ptr<costmap_2d::Layer>>* plugins = costmap2d_ros_->getLayeredCostmap()->getPlugins();

    ROS_INFO_STREAM("==== pre assert");
    // assert only have one layer
    assert((*plugins).size() == 1);

    ROS_INFO_STREAM("==== post assert");

    // get the old
    // dynamic_cast<costmap_2d::Costmap2D*>(*plugins[0].get())->getCharMap()
    auto obstacle_layer = static_cast<costmap_2d::CostmapLayer*>((*plugins)[0].get());

    // the copy of the old obstacle map
    costmap_2d::Costmap2D* costmap_copy = new costmap_2d::Costmap2D(*obstacle_layer);

    // old size
    int old_size_x = costmap_copy->getSizeInCellsX();
    int old_size_y = costmap_copy->getSizeInCellsY();

    // resize the map (layeredCostmap &-> plugins)
    int new_size_x = (new_max_x - new_min_x) / resolution_;
    int new_size_y = (new_max_y - new_min_y) / resolution_;
    
    costmap2d_ros_->getLayeredCostmap()->resizeMap(new_size_x, new_size_y, resolution_, new_min_x, new_min_y);

    ROS_INFO_STREAM("====== old size " << old_size_x << " x " << old_size_y << " / new size " << new_size_x << " x " << new_size_y);
    // claim the mutex
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(obstacle_layer->getMutex()));

    // now all layers are reset, only need to copy the old obstacle layer
    int x_offset = (min_x_ - new_min_x) / resolution_;
    int y_offset = (min_y_ - new_min_y) / resolution_;

    ROS_INFO_STREAM("====== new min " << new_min_x << "," << new_min_y);
    ROS_INFO_STREAM("====== old min " << min_x_ << "," << min_y_);

    ROS_INFO_STREAM("====== offset x " << x_offset << ", y " << y_offset);

    for (unsigned int i = 0; i < old_size_x; i++) {
        for (unsigned int j = 0; j < old_size_y; j++) {
            int new_i = i + x_offset;
            int new_j = j + y_offset;
            // ROS_INFO_STREAM("====== old i, j " << i << " , " << j << " / new i, j " << new_i << " , " << new_j << " value : " << int(costmap_copy->getCost(i,j)));            
            // set to the obstacle layer
            obstacle_layer->setCost(new_i, new_j, costmap_copy->getCost(i,j));
        }
    }

    // need to manually propagate the update to the master map, 
    // since the update bound might not fully enclose the transcription and 
    // the full transcription thus will not reflected in the next update in the main loop
    costmap_2d::Costmap2D* new_costmap = costmap2d_ros_->getCostmap();
    ROS_INFO_STREAM("====== old size " << old_size_x << " x " << old_size_y << " / new size " << new_size_x << " x " << new_size_y);
    for (unsigned int i = 0; i < old_size_x; i++) {
        for (unsigned int j = 0; j < old_size_y; j++) {
            int new_i = i + x_offset;
            int new_j = j + y_offset;
            // ROS_INFO_STREAM("====== old i, j " << i << " , " << j << " / new i, j " << new_i << " , " << new_j << " value : " << int(costmap_copy->getCost(i,j)));            
            // set to the master map
            new_costmap->setCost(new_i, new_j, costmap_copy->getCost(i,j));
        }
    }

    // update the new map bounds
    min_x_ = new_min_x;
    min_y_ = new_min_y;
    max_x_ = new_max_x;
    max_y_ = new_max_y;

}

} //namespace smb_exploration