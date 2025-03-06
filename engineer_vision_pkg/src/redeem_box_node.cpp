#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

class redeem_box_node :public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    cv_bridge::CvImagePtr framePtr;
    cv::Mat frame, element, frame_preprocessed, frame_copy;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    int hmin = 17, smin = 23, vmin = 246;
    int hmax = 44, smax = 97, vmax = 255;
    double contourArea;
    float peri;
    std::vector<cv::Point> Points;

public:
    explicit redeem_box_node(const std::string &node_name) : Node(node_name)
    {
        RCLCPP_INFO(get_logger(), "redeem_box_node launched");
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("processed_image", 10);
        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("redeem_box_image", 10, [&](const sensor_msgs::msg::Image::SharedPtr msg) ->void {
            image_preprocess(msg);
        });
    }

    void image_preprocess(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        framePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        frame = framePtr->image;
        frame_copy = frame.clone();
        cv::cvtColor(frame, frame, cv::COLOR_BGR2HSV);
        cv::Scalar lowerHSV(hmin, smin, vmin);
        cv::Scalar upperHSV(hmax, smax, vmax);
        cv::inRange(frame, lowerHSV, upperHSV, frame);
        element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(13, 13));

        dilate(frame, frame, element);
        cv::findContours(frame, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        RCLCPP_INFO(get_logger(), "Found %lu contours", contours.size());
        
        std::vector<std::vector<cv::Point>> conPoly(contours.size());
        std::vector<std::vector<cv::Point>> triangle;
        Points.clear();
        
        for (size_t i = 0; i < contours.size(); i++)
        {   
            contourArea = cv::contourArea(contours[i]);
            if (contourArea > 1500 || contourArea < 1550)
            {
                peri = cv::arcLength(contours[i], true);
                cv::approxPolyDP(contours[i], conPoly[i], 0.02*peri, true);
                
                std::vector<cv::Point> current_triangle;
                cv::minEnclosingTriangle(conPoly[i], current_triangle);
                
                if (current_triangle.size() == 3)
                {
                    triangle.push_back(current_triangle);
                    RCLCPP_INFO(get_logger(), "Triangle %zu vertices:", i);
                    for (int j = 0; j < 3; j++) {
                        RCLCPP_INFO(get_logger(), "%d: (%d, %d)", j, current_triangle[j].x, current_triangle[j].y);
                    }
                }
            }
        }
        
        for (size_t i = 0; i < triangle.size(); i++)
        {
            double length0_square = (triangle[i][1].x - triangle[i][2].x)*(triangle[i][1].x - triangle[i][2].x) + 
                                  (triangle[i][1].y - triangle[i][2].y)*(triangle[i][1].y - triangle[i][2].y);
            double length1_square = (triangle[i][0].x - triangle[i][2].x)*(triangle[i][0].x - triangle[i][2].x) + 
                                  (triangle[i][0].y - triangle[i][2].y)*(triangle[i][0].y - triangle[i][2].y);
            double length2_square = (triangle[i][1].x - triangle[i][0].x)*(triangle[i][1].x - triangle[i][0].x) + 
                                  (triangle[i][1].y - triangle[i][0].y)*(triangle[i][1].y - triangle[i][0].y);
            
            RCLCPP_INFO(get_logger(), "Triangle %zu lengths: %f, %f, %f", i, length0_square, length1_square, length2_square);
            
            if (length0_square > length1_square && length0_square > length2_square)
            {
                Points.push_back(triangle[i][0]);
                RCLCPP_INFO(get_logger(), "Selected point 0");
            }
            else if (length1_square > length0_square && length1_square > length2_square)
            {
                Points.push_back(triangle[i][1]);
                RCLCPP_INFO(get_logger(), "Selected point 1");
            }
            else if (length2_square > length0_square && length2_square > length1_square)
            {
                Points.push_back(triangle[i][2]);
                RCLCPP_INFO(get_logger(), "Selected point 2");
            }
        }
        
        for (const auto& point : Points)
        {
            cv::circle(frame_copy, point, 5, cv::Scalar(0, 255, 0), cv::FILLED);
        }
        
        auto msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_copy).toImageMsg();
        publisher_->publish(*msg_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<redeem_box_node>("redeem_box_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}