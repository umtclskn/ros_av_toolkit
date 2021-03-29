#include "lidar/bounding_box.hpp"


#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;


namespace lidar
{
    BoundingBox::BoundingBox(const rclcpp::NodeOptions & options): Node("bounding_box", options)
    {
        RCLCPP_INFO(get_logger(), "BoundingBox Init");
        this->init_parameters();

        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(publisher_url_, 10);

        subscription_ = this->create_subscription<av_toolkit_custom_msgs::msg::Clusters>(
            subscription_url_,
            10,
            std::bind(&BoundingBox::on_bounding_box, this, std::placeholders::_1));
    }


    void BoundingBox::on_bounding_box(const av_toolkit_custom_msgs::msg::Clusters::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Bounding Box cluster size: %i", msg->pointclouds.size());

        std::vector<sensor_msgs::msg::PointCloud2> clusters;
        clusters= msg->pointclouds;

        marker_array_.markers.clear();

        for (auto & cloud_item : clusters) {
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(cloud_item, pcl_pc2);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(pcl_pc2,*cloud_filtered);

            pcl::PointXYZ minPt, maxPt;
            pcl::getMinMax3D (*cloud_filtered, minPt, maxPt);

            visualization_msgs::msg::Marker marker;

            static int id = 1;
            marker.header.frame_id = "base_link";
            marker.header.stamp = this->now();
            marker.ns = "marker";
            marker.id = id++; //unused
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1;
            marker.scale.x = maxPt.x - minPt.x;
            marker.scale.y = maxPt.y - minPt.y;
            marker.scale.z = 1;
            marker.color.a = 0.80;
            marker.color.r = 0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.lifetime = rclcpp::Duration(100ms);
            marker.pose.position.x = minPt.x;
            marker.pose.position.y = minPt.y;
            marker.pose.position.z = 0;
            
            marker_array_.markers.push_back(marker);

            RCLCPP_INFO(get_logger(), "Bounding Box min x: %f", minPt.x);
        }

        publisher_->publish(marker_array_);
    }

    void BoundingBox::init_parameters()
    {
        RCLCPP_INFO(get_logger(), "############   BOUNDINGBOX START    ##########");

        this->declare_parameter<std::string>("publisher_url", "perception/lidar/bounding_box");
        this->declare_parameter<std::string>("subscription_url", "perception/lidar/cluster");

        this->get_parameter("publisher_url", publisher_url_);
        RCLCPP_INFO(get_logger(), "perception lidar bounding_box publisher_url: %s", publisher_url_.c_str());

        this->get_parameter("subscription_url", subscription_url_);
        RCLCPP_INFO(get_logger(), "perception lidar bounding_box subscription_url: %s", subscription_url_.c_str());

        RCLCPP_INFO(get_logger(), "############   BOUNDINGBOX END     ##########");
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(lidar::BoundingBox)
