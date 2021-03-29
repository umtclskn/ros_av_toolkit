#include "lidar/ground_filter.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace lidar
{
  GroundFilter::GroundFilter(const rclcpp::NodeOptions & options): Node("ground_filter", options), count_(0)
  {
    // read parameters from preprocess.yaml file
    this->init_parameters();

    publisher_inliers_ = create_publisher<sensor_msgs::msg::PointCloud2>(publisher_inliers_url_, 10);
    publisher_outliers_ = create_publisher<sensor_msgs::msg::PointCloud2>(publisher_outliers_url_, 10);

    subscription_ =     this->create_subscription<sensor_msgs::msg::PointCloud2>(
        subscription_url_,
        10,
        std::bind(&GroundFilter::on_gound_filter, this, std::placeholders::_1));
  }

  void GroundFilter::on_gound_filter(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

    std::unordered_set<int> inliersResult;


      pcl::PointIndices::Ptr inliers(new pcl::PointIndices); // Build on the heap
      // TODO:: Fill in this function to find inliers for the cloud.
      pcl::ModelCoefficients::Ptr coefficient(new pcl::ModelCoefficients);
      pcl::SACSegmentation<pcl::PointXYZI> seg;

      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setMaxIterations(max_iteration_);
      seg.setDistanceThreshold(distance_tolerance_); 

  
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud(cloud);
      seg.segment(*inliers, *coefficient);

      // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
      pcl::PointCloud<pcl::PointXYZI>::Ptr obstCloud(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZI>());

      for (int index : inliers->indices) {
          planeCloud->points.push_back(cloud->points[index]);
      }
      // create extraction object
      pcl::ExtractIndices<pcl::PointXYZI> extract;
      extract.setInputCloud(cloud);
      extract.setIndices(inliers);
      extract.setNegative(true);
      extract.filter(*obstCloud);


      // Convert to ROS data type
      sensor_msgs::msg::PointCloud2 output_inliers;
      sensor_msgs::msg::PointCloud2 output_outliers;


      pcl::PCLPointCloud2 local_inliners_ptr;
      pcl::toPCLPointCloud2(*planeCloud, local_inliners_ptr);
      pcl_conversions::fromPCL(local_inliners_ptr, output_inliers);
      output_inliers.header.frame_id = "base_link";
      output_inliers.header.stamp = now();
      publisher_inliers_->publish(output_inliers);

      pcl::PCLPointCloud2 local_outliers_ptr;
      pcl::toPCLPointCloud2(*obstCloud, local_outliers_ptr);
      pcl_conversions::fromPCL(local_outliers_ptr, output_outliers);
      output_outliers.header.frame_id = "base_link";
      output_outliers.header.stamp = now();
      publisher_outliers_->publish(output_outliers);
  }

  void GroundFilter::init_parameters(){
    RCLCPP_INFO(get_logger(), "############   GROUND_FILTER START    ##########");
    this->declare_parameter<std::string>("publisher_inliers_url", "perception/lidar/ground_filter_inliers");
    this->declare_parameter<std::string>("publisher_outliers_url", "perception/lidar/ground_filter_outliers");
    this->declare_parameter<std::string>("subscription_url", "perception/lidar/preprocess");
    this->declare_parameter<int>("max_iteration", 48);
    this->declare_parameter<double>("distance_tolerance", 0.3);

    this->get_parameter("publisher_inliers_url", publisher_inliers_url_);
    RCLCPP_INFO(get_logger(), "perception lidar groung filter publisher_inliers_url: %s", publisher_inliers_url_.c_str());

    this->get_parameter("publisher_outliers_url", publisher_outliers_url_);
    RCLCPP_INFO(get_logger(), "perception lidar groung filter publisher_outliers_url: %s", publisher_outliers_url_.c_str());

    this->get_parameter("subscription_url", subscription_url_);
    RCLCPP_INFO(get_logger(), "perception lidar groung filter subscription_url: %s", subscription_url_.c_str());

    this->get_parameter("max_iteration", max_iteration_);
    RCLCPP_INFO(get_logger(), "perception lidar groung filter max_iteration: %i", max_iteration_);

    this->get_parameter("distance_tolerance", distance_tolerance_);
    RCLCPP_INFO(get_logger(), "perception lidar groung filter distance_tolerance: %f", distance_tolerance_);
    RCLCPP_INFO(get_logger(), "############   GROUND_FILTER END    ##########");
  }

}  // namespace lidar

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(lidar::GroundFilter)
