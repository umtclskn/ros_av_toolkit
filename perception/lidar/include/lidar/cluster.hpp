
#ifndef LIDAR__CLUSTER_HPP_
#define LIDAR__CLUSTER_HPP_

#include "lidar/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "av_toolkit_custom_msgs/msg/clusters.hpp"

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>



namespace lidar
{

class Cluster : public rclcpp::Node
{
public:
  PERCEPTION_LIDAR_PUBLIC
  explicit Cluster(const rclcpp::NodeOptions & options);

protected:
  void on_cluster(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  size_t count_;
  rclcpp::Publisher<av_toolkit_custom_msgs::msg::Clusters>::SharedPtr publisher_cluster_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_cluster_merged_points_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  std::string publisher_cluster_url_;
  std::string publisher_cluster_merged_points_url_;
  std::string subscription_url_;

  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;

  void init_parameters();

};

}  // namespace lidar

#endif  // LIDAR__CLUSTER_HPP_
