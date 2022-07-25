#include "line_follower/image_processed.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "opencv2/highgui.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace line_follower
{
    ImageProcessed::ImageProcessed(const rclcpp::NodeOptions &options)
        : Node("image_processed", options)
    {
        img_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "image",
            10,
            std::bind(&ImageProcessed::image_callback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "ImageProcessed node ready");
    }

    void ImageProcessed::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            img = cv_ptr->image;
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
    }

    void ImageProcessed::process_image()
    {
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
        cv::threshold(img, img, 127, 255, cv::THRESH_BINARY);
        cv::imshow("Image", img);
        cv::waitKey(1);
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(line_follower::ImageProcessed)