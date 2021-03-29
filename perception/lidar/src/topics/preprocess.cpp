#include "lidar/preprocess.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;


namespace lidar
{
  Preprocess::Preprocess(const rclcpp::NodeOptions & options): Node("preprocess", options), count_(0)
  {
    // read parameters from preprocess.yaml file
    this->init_parameters();

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(publisher_url_, 10);

    subscription_ =     this->create_subscription<sensor_msgs::msg::PointCloud2>(
        subscription_url_,
        10,
        std::bind(&Preprocess::on_timer_subscription, this, std::placeholders::_1));
  }

  void Preprocess::on_timer_subscription(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "preprocess on_timer_subscription start %i", msg->width);

    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*msg, *cloud);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (leaf_size_, leaf_size_, leaf_size_);
    sor.filter (cloud_filtered);

    pcl::PCLPointCloud2* cloud_ptr = new pcl::PCLPointCloud2(cloud_filtered); 
    pcl::PCLPointCloud2ConstPtr cloud_ptr_roi(cloud_ptr);
    pcl::PCLPointCloud2 cloud_filtered_roi;
    pcl::CropBox<pcl::PCLPointCloud2> region(region_extract_removed_indices_);
    region.setMin(region_min_vec_);
    region.setMax(region_max_vec_);
    region.setInputCloud(cloud_ptr_roi);
    region.filter(cloud_filtered_roi);

    pcl::PCLPointCloud2* cloud_roof_ptr = new pcl::PCLPointCloud2(cloud_filtered_roi); 
    pcl::PCLPointCloud2ConstPtr cloud_roof_ptr_roi(cloud_roof_ptr);
    pcl::PCLPointCloud2 cloud_roof_filtered_roi;
    pcl::CropBox<pcl::PCLPointCloud2> region_roof;
    region_roof.setMin(region_roof_min_vec_);
    region_roof.setMax(region_roof_max_vec_);
    region_roof.setNegative(region_roof_negative_bool_);
    region_roof.setInputCloud(cloud_roof_ptr_roi);
    region_roof.filter(cloud_roof_filtered_roi);

    // Convert to ROS data type
    sensor_msgs::msg::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_roof_filtered_roi, output);

    publisher_->publish(output);
    
    RCLCPP_INFO(get_logger(), "preprocess on_timer_subscription end");
  }

  void Preprocess::init_parameters(){

    RCLCPP_INFO(get_logger(), "############   PREPROCESS START    ##########");

    this->declare_parameter<std::string>("publisher_url","perception/lidar/preprocess");
    this->declare_parameter<std::string>("subscription_url","kitti/point_cloud");
    this->declare_parameter<double>("leaf_size", 0.3);
    this->declare_parameter<bool>("region_extract_removed_indices", true);
    this->declare_parameter<bool>("region_roof_negative_bool", true);
    this->declare_parameter<std::vector<double>>("region_min", std::vector<double>{ -20.0, -6.0, -2.0, 1.0 });
    this->declare_parameter<std::vector<double>>("region_max", std::vector<double>{ 30.0, 7.0, 5.0, 1.0 });
    this->declare_parameter<std::vector<double>>("region_roof_min", std::vector<double>{ -1.5, -1.4, -2.0, 1.0 });
    this->declare_parameter<std::vector<double>>("region_roof_max", std::vector<double>{ 2.7, 1.4, 0.0, 1.0 });
    

    rclcpp::Parameter param_leaf_size = this->get_parameter("leaf_size");
    rclcpp::Parameter param_region_extract_removed_indices = this->get_parameter("region_extract_removed_indices");
    rclcpp::Parameter param_region_roof_negative_bool = this->get_parameter("region_roof_negative_bool");
    rclcpp::Parameter param_region_min = this->get_parameter("region_min");
    rclcpp::Parameter param_region_max = this->get_parameter("region_max");
    rclcpp::Parameter param_region_roof_min = this->get_parameter("region_roof_min");
    rclcpp::Parameter param_region_roof_max = this->get_parameter("region_roof_max");
 
    this->get_parameter("publisher_url", publisher_url_);
    RCLCPP_INFO(get_logger(), "perception lidar preprocess publisher_url_: %s", publisher_url_.c_str());

    this->get_parameter("subscription_url", subscription_url_);
    RCLCPP_INFO(get_logger(), "perception lidar preprocess subscription_url_: %s", subscription_url_.c_str());

    leaf_size_ = (float) param_leaf_size.as_double();

    // region
    std::vector<double> region_min_double = param_region_min.as_double_array();
    std::vector<float> region_min_float_vec(region_min_double.begin(), region_min_double.end());
    region_min_vec_= Eigen::Map<Eigen::Vector4f, Eigen::Unaligned>(region_min_float_vec.data(), region_min_float_vec.size());

    std::vector<double> region_max_double = param_region_max.as_double_array();
    std::vector<float> region_max_float_vec(region_max_double.begin(), region_max_double.end());
    region_max_vec_= Eigen::Map<Eigen::Vector4f, Eigen::Unaligned>(region_max_float_vec.data(), region_max_float_vec.size());

    region_extract_removed_indices_ = param_region_extract_removed_indices.as_bool();

    // region roof
    std::vector<double> region_roof_min_double = param_region_roof_min.as_double_array();
    std::vector<float> region_roof_min_float_vec(region_roof_min_double.begin(), region_roof_min_double.end());
    region_roof_min_vec_ = Eigen::Map<Eigen::Vector4f, Eigen::Unaligned>(region_roof_min_float_vec.data(), region_roof_min_float_vec.size());

    std::vector<double> region_roof_max_double = param_region_roof_max.as_double_array();
    std::vector<float> region_roof_max_float_vec(region_roof_max_double.begin(), region_roof_max_double.end());
    region_roof_max_vec_ = Eigen::Map<Eigen::Vector4f, Eigen::Unaligned>(region_roof_max_float_vec.data(), region_roof_max_float_vec.size());

    region_roof_negative_bool_ = param_region_roof_negative_bool.as_bool();
    
    RCLCPP_INFO(get_logger(), "############   PREPROCESS END    ##########");
  }

}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(lidar::Preprocess)