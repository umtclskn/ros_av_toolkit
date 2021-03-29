
#ifndef LIDAR__GROUND_FILTER_HPP_
#define LIDAR__GROUND_FILTER_HPP_

#include "lidar/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <unordered_set>



namespace lidar
{

class GroundFilter : public rclcpp::Node
{
public:
  PERCEPTION_LIDAR_PUBLIC
  explicit GroundFilter(const rclcpp::NodeOptions & options);

protected:
  void on_gound_filter(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  size_t count_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_inliers_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_outliers_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  std::string publisher_inliers_url_;
  std::string publisher_outliers_url_;
  std::string subscription_url_;

  int max_iteration_;
  double distance_tolerance_;

  void init_parameters();
};

}  // namespace lidar

#endif  // LIDAR__GROUND_FILTER_HPP_
