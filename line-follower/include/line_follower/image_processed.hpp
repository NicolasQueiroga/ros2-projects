#ifndef __IMAGE_PROCESSED_HPP_
#define __IMAGE_PROCESSED_HPP_

#include "line_follower/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

namespace line_follower
{
    class ImageProcessed : public rclcpp::Node
    {
    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;
        cv::Mat img;

    public:
        COMPOSITION_PUBLIC
        explicit ImageProcessed(const rclcpp::NodeOptions &options);

    protected:
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void process_image();
    };
}

#endif