
#ifndef LIDAR__BOUNDING_BOX_HPP_
#define LIDAR__BOUNDING_BOX_HPP_

#include "lidar/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "av_toolkit_custom_msgs/msg/clusters.hpp"

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>

namespace lidar
{
    class BoundingBox : public rclcpp::Node
    {
        public:
            PERCEPTION_LIDAR_PUBLIC
            explicit BoundingBox(const rclcpp::NodeOptions & options);

        protected:
            void on_bounding_box(const av_toolkit_custom_msgs::msg::Clusters::SharedPtr msg);

        private:
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
            rclcpp::Subscription<av_toolkit_custom_msgs::msg::Clusters>::SharedPtr subscription_;

            visualization_msgs::msg::MarkerArray marker_array_;

            std::string publisher_url_;
            std::string subscription_url_;

            void init_parameters();
    };

}  // namespace lidar

#endif  // LIDAR__BOUNDING_BOX_HPP_
