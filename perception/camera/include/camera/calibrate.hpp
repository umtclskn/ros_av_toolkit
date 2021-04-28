
#ifndef CAMERA__CALIBRATE_HPP_
#define CAMERA__CALIBRATE_HPP_

#include "camera/visibility.h"
#include "rclcpp/rclcpp.hpp"

#include <ros_av_interfaces/srv/camera_info.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>


#include <sensor_msgs/msg/image.hpp>

#include <fstream>
#include <filesystem>
#include <vector>
#include <string>
#include <cstdlib>
#include <map>

namespace camera
{

class Calibrate : public rclcpp::Node
{
public:
  PERCEPTION_CAMERA_PUBLIC
  explicit Calibrate(const rclcpp::NodeOptions & options);

protected:
  void on_calibrate(const sensor_msgs::msg::Image::SharedPtr msg);
  ros_av_interfaces::srv::CameraInfo::Response::SharedPtr send_request(ros_av_interfaces::srv::CameraInfo::Request::SharedPtr request);
  int encoding2mat_type(std::string encoding);
private:
  size_t count_;

  rclcpp::Client<ros_av_interfaces::srv::CameraInfo>::SharedPtr client_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;


  std::string publisher_url_;
  std::string subscription_url_;
  std::string service_url_;

  int service_request_cam_type_;

  sensor_msgs::msg::CameraInfo camera_info_;

  void init_parameters();
};

}  // namespace camera

#endif  // CAMERA__CALIBRATE_HPP_
