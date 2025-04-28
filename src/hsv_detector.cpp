#include "rclcpp/rclcpp.hpp"
#include "opencv4/opencv2/highgui.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <thread>

int hmin = 0, smin = 0, vmin = 0;
int hmax = 255, smax = 255, vmax = 255;

void empty(int, void*) {}

class hsv_detector_node : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    cv_bridge::CvImagePtr framePtr;
    cv::Mat frame, frame_hsv, mask;
    

public:
    hsv_detector_node(const std::string node_name) : rclcpp::Node(node_name)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("hsv_image", 10);
        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("redeem_box_image", 10, [&](const sensor_msgs::msg::Image::SharedPtr msg) ->void {
            framePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            frame = framePtr->image;
            cv::cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV);
            hmin = cv::getTrackbarPos("Hue Min", "Trackbars");
            hmax = cv::getTrackbarPos("Hue Max", "Trackbars");
            smin = cv::getTrackbarPos("Sat Min", "Trackbars");
            smax = cv::getTrackbarPos("Sat Max", "Trackbars");
            vmin = cv::getTrackbarPos("Val Min", "Trackbars");
            vmax = cv::getTrackbarPos("Val Max", "Trackbars");
            cv::Scalar lowerHSV(hmin, smin, vmin);
            cv::Scalar upperHSV(hmax, smax, vmax);
            cv::inRange(frame_hsv, lowerHSV, upperHSV, mask);
            auto msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask).toImageMsg();
            this->publisher_->publish(*msg_);
        });
    }
};


int main (int argc, char** argv)
{
    cv::namedWindow("Trackbars", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("Hue Min", "Trackbars", nullptr, 179, empty);
    cv::createTrackbar("Hue Max", "Trackbars", nullptr, 179, empty);
    cv::createTrackbar("Sat Min", "Trackbars", nullptr, 255, empty);
    cv::createTrackbar("Sat Max", "Trackbars", nullptr, 255, empty);
    cv::createTrackbar("Val Min", "Trackbars", nullptr, 255, empty);
    cv::createTrackbar("Val Max", "Trackbars", nullptr, 255, empty);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<hsv_detector_node>("hsv_detector_node");

    std::thread ros_thread([&]() {
        rclcpp::spin(node);
    });

    while (rclcpp::ok())
    {
        cv::waitKey(1);
    }

    ros_thread.join();
    rclcpp::shutdown();
    cv::destroyAllWindows();
}