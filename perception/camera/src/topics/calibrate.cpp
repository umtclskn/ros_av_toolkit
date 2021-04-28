#include "camera/calibrate.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std::chrono_literals;


namespace camera
{
    Calibrate::Calibrate(const rclcpp::NodeOptions & options): Node("calibrate", options), count_(0)
    {
        // read parameters from calibrate.yaml file
        this->init_parameters();

        publisher_ = create_publisher<sensor_msgs::msg::Image>(publisher_url_, 10);

        client_ = this->create_client<ros_av_interfaces::srv::CameraInfo>(service_url_);

        auto request = std::make_shared<ros_av_interfaces::srv::CameraInfo::Request>();
        request->cam_num  = 2;

        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
            }
            RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
        }

        auto result = send_request(request);
        if (result) {
            RCLCPP_INFO(this->get_logger(), "Result of add_two_ints: %i", result->success);
            camera_info_ = result->camera_info;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for response. Exiting.");
        }

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            subscription_url_,
            10,
            std::bind(&Calibrate::on_calibrate, this, std::placeholders::_1));

    }

    void Calibrate::on_calibrate(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received image #%s", msg->header.frame_id.c_str());
        
        // Convert to an OpenCV matrix by assigning the data.
        cv::Mat frame(
            msg->height, msg->width, encoding2mat_type(msg->encoding),
            const_cast<unsigned char *>(msg->data.data()), msg->step);

        if (msg->encoding == "rgb8") {
            cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
        }

        cv::Mat k(3, 3, CV_64FC1, (void *) camera_info_.k.data());
        cv::Mat r(3, 3, CV_64FC1, (void *) camera_info_.r.data());
        cv::Mat p(4, 3, CV_64FC1, (void *) camera_info_.p.data());
        cv::Mat d(1, 5, CV_64FC1, (void *) camera_info_.d.data());

        cv::Mat undistort_image;

        cv::Mat cvframe = frame;

        cv::undistort(cvframe, undistort_image, k, d);

        // Show the image in a window
        cv::imshow("calibrate", undistort_image);
        // Draw the screen and wait for 1 millisecond.
        cv::waitKey(1);
    }

    int Calibrate::encoding2mat_type(std::string encoding)
    {
        if (encoding == "mono8") {
            return CV_8UC1;
        } else if (encoding == "bgr8") {
            return CV_8UC3;
        } else if (encoding == "mono16") {
            return CV_16SC1;
        } else if (encoding == "rgba8") {
            return CV_8UC4;
        } else if (encoding == "bgra8") {
            return CV_8UC4;
        } else if (encoding == "32FC1") {
            return CV_32FC1;
        } else if (encoding == "rgb8") {
            return CV_8UC3;
        } else {
            throw std::runtime_error("Unsupported encoding type");
        }
    }

    void Calibrate::init_parameters(){
        RCLCPP_INFO(get_logger(), "############   CALIBRATE PARAMETERS START    ##########");
        this->declare_parameter<std::string>("publisher_url","perception/camera/calibrated_image");
        this->declare_parameter<std::string>("subscription_url","kitti/image/color/left");
        this->declare_parameter<std::string>("service_url","ros_av_interfaces/camera_info");
        this->declare_parameter<int>("service_request_cam_type", 2);
        
        this->get_parameter("publisher_url", publisher_url_);
        RCLCPP_INFO(get_logger(), "perception camera calibrate publisher_url_: %s", publisher_url_.c_str());

        this->get_parameter("subscription_url", subscription_url_);
        RCLCPP_INFO(get_logger(), "perception camera calibrate subscription_url_: %s", subscription_url_.c_str());

        this->get_parameter("service_url", service_url_);
        RCLCPP_INFO(get_logger(), "perception camera calibrate service_url_: %s", service_url_.c_str());

        this->get_parameter("service_request_cam_type", service_request_cam_type_);
        RCLCPP_INFO(get_logger(), "perception camera calibrate service_request_cam_type: %i", service_request_cam_type_);

        RCLCPP_INFO(get_logger(), "############   CALIBRATE PARAMETERS END    ##########");
    }

    ros_av_interfaces::srv::CameraInfo::Response::SharedPtr Calibrate::send_request(ros_av_interfaces::srv::CameraInfo::Request::SharedPtr request)
    {
        auto result = client_->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return result.get();
        } else {
            return NULL;
        }
    }


}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(camera::Calibrate)