
#ifndef LIDAR__CLUSTER_HPP_
#define LIDAR__CLUSTER_HPP_

#include "lidar/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>


namespace lidar
{

class Preprocess : public rclcpp::Node
{
public:
  PERCEPTION_LIDAR_PUBLIC
  explicit Preprocess(const rclcpp::NodeOptions & options);

protected:
  void on_timer_subscription(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  size_t count_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  std::string publisher_url_;
  std::string subscription_url_;

  float leaf_size_;

  Eigen::Vector4f region_min_vec_;
  Eigen::Vector4f region_max_vec_;
  bool region_extract_removed_indices_;


  Eigen::Vector4f region_roof_min_vec_;
  Eigen::Vector4f region_roof_max_vec_;
  bool region_roof_negative_bool_;

  void init_parameters();

};

}  // namespace lidar

#endif  // LIDAR__PREPROCESS_HPP_
