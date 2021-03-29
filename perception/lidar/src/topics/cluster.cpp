#include "lidar/cluster.hpp"


#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace lidar
{
  Cluster::Cluster(const rclcpp::NodeOptions & options): Node("cluster", options), count_(0)
  {
    this->init_parameters();

    publisher_cluster_ = create_publisher<av_toolkit_custom_msgs::msg::Clusters>(publisher_cluster_url_, 10);
    publisher_cluster_merged_points_ = create_publisher<sensor_msgs::msg::PointCloud2>(publisher_cluster_merged_points_url_, 10);

    subscription_ =     this->create_subscription<sensor_msgs::msg::PointCloud2>(
        subscription_url_,
        10,
        std::bind(&Cluster::on_cluster, this, std::placeholders::_1));
  }

  void Cluster::on_cluster(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto startTime = std::chrono::steady_clock::now();

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud_filtered);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (cluster_tolerance_); // 2cm
    ec.setMinClusterSize (min_cluster_size_);
    ec.setMaxClusterSize (max_cluster_size_);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_point_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<sensor_msgs::msg::PointCloud2> clusters;
    int i = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*

      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      *merged_point_cloud += *cloud_cluster;

      sensor_msgs::msg::PointCloud2 local_point_cloud_msg;
      pcl::PCLPointCloud2 local_inliners_ptr;
      pcl::toPCLPointCloud2(*cloud_cluster, local_inliners_ptr);
      pcl_conversions::fromPCL(local_inliners_ptr, local_point_cloud_msg);

      clusters.push_back(local_point_cloud_msg);
      i++;
    }
    RCLCPP_INFO(get_logger(), "clusters size : %i", i);

    sensor_msgs::msg::PointCloud2 output_merged_clusters;
    pcl::PCLPointCloud2 local_ptr;
    pcl::toPCLPointCloud2(*merged_point_cloud, local_ptr);
    pcl_conversions::fromPCL(local_ptr, output_merged_clusters);
    output_merged_clusters.header.frame_id = "base_link";
    output_merged_clusters.header.stamp =  rclcpp::Time();
    publisher_cluster_merged_points_->publish(output_merged_clusters);

    av_toolkit_custom_msgs::msg::Clusters output_cluster;
    output_cluster.header.frame_id = "base_link";
    output_cluster.header.stamp = rclcpp::Time();
    output_cluster.pointclouds = clusters;
    publisher_cluster_->publish(output_cluster);
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Cluster took: " << elapsedTime.count() << " milliseconds" << std::endl;
  }

  void Cluster::init_parameters(){
    RCLCPP_INFO(get_logger(), "############   CLUSTER START    ##########");
    
    this->declare_parameter<std::string>("publisher_cluster_url", "perception/lidar/cluster");
    this->declare_parameter<std::string>("publisher_cluster_merged_points_url", "perception/lidar/cluster_merged_points");
    this->declare_parameter<std::string>("subscription_url", "perception/lidar/ground_filter_outliers");
    this->declare_parameter<double>("cluster_tolerance", 0.5);
    this->declare_parameter<int>("min_cluster_size", 10);
    this->declare_parameter<int>("max_cluster_size", 140);

    this->get_parameter("publisher_cluster_url", publisher_cluster_url_);
    RCLCPP_INFO(get_logger(), "perception lidar cluster publisher_cluster_url: %s", publisher_cluster_url_.c_str());

    this->get_parameter("publisher_cluster_merged_points_url", publisher_cluster_merged_points_url_);
    RCLCPP_INFO(get_logger(), "perception lidar cluster publisher_cluster_merged_points_url: %s", publisher_cluster_merged_points_url_.c_str());

    this->get_parameter("subscription_url", subscription_url_);
    RCLCPP_INFO(get_logger(), "perception lidar cluster subscription_url: %s", subscription_url_.c_str());

    this->get_parameter("cluster_tolerance", cluster_tolerance_);
    RCLCPP_INFO(get_logger(), "perception lidar cluster cluster_tolerance: %d", cluster_tolerance_);

    this->get_parameter("min_cluster_size", min_cluster_size_);
    RCLCPP_INFO(get_logger(), "perception lidar cluster min_cluster_size: %i", min_cluster_size_);

    this->get_parameter("max_cluster_size", max_cluster_size_);
    RCLCPP_INFO(get_logger(), "perception lidar cluster max_cluster_size: %i", max_cluster_size_);
    RCLCPP_INFO(get_logger(), "############   CLUSTER END    ##########");
  }
}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(lidar::Cluster)
