#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include <fstream>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 
#include "tf2/LinearMath/Quaternion.h"             
#include "tf2_ros/transform_broadcaster.h" 
#include "serial/serial.h"

class redeem_box_node :public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    cv_bridge::CvImagePtr framePtr;
    cv::Mat frame, element, frame_preprocessed, frame_copy;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    int hmin = 17, smin = 23, vmin = 246;
    int hmax = 44, smax = 97, vmax = 255;
    double contourArea;
    float peri;
    std::vector<std::vector<cv::Point>> conPoly;
    std::vector<std::vector<cv::Point2f>> triangle;
    std::vector<cv::Point2f> circle;
    std::vector<float> radius;
    std::vector<cv::Point> Points;
    std::queue<std::vector<cv::Point>> point_queue;
    cv::Point2f center;
    cv::Mat camera_matrix, distortion_coefficients, rectification_matrix, projection_matrix, rvec, tvec;
    std::vector<cv::Point3f> objPoints;
    geometry_msgs::msg::TransformStamped transformation;
    tf2::Quaternion q;
    double roll, pitch, yaw;

public:
    explicit redeem_box_node(const std::string &node_name) : Node(node_name)
    {
        RCLCPP_INFO(get_logger(), "redeem_box_node launched");
        loadCameraParams();
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("processed_image", 10);
        this->broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("redeem_box_image", 10, [&](const sensor_msgs::msg::Image::SharedPtr msg) ->void {
            image_preprocess(msg);
            publish_tf(tvec, roll, pitch, yaw);
        });

    }

    void loadCameraParams()
    {
        cv::FileStorage fs("/home/ooofieee/Code/ws_0/src/engineer_vision_pkg/config/test.yaml", cv::FileStorage::READ);
        fs["camera_matrix"] >> camera_matrix;
        fs["distortion_coefficients"] >> distortion_coefficients;
        fs["rectification_matrix"] >> rectification_matrix;
        fs["projection_matrix"] >> projection_matrix;
        fs.release();
    }

    std::vector<cv::Point> filter (std::vector<cv::Point> & points)
    {
        std::vector<cv::Point> point_temp(4);
        if (point_queue.size() < 2)
        {
            point_queue.push(points);
        }
        else
        {
            point_queue.pop();
            point_queue.push(points);
        }

        if (point_queue.size() == 2)
        {
            for(size_t i = 0; i < point_queue.back().size(); i++)
            {
                point_temp[i] = point_queue.back()[i] * 0.2 + point_queue.front()[i] * 0.8;
            }
            return point_temp;
        }
        else
        {
            return point_temp;
        }
    }

    void pnpSolver()
    {
        bool succees = solvePnP(objPoints, Points, camera_matrix, distortion_coefficients, rvec, tvec, cv::SOLVEPNP_IPPE_SQUARE);
        if (!succees)
        {
            cv::Mat obj2Cam = cv::Mat::zeros(4, 4, CV_64FC1);
            Rodrigues(rvec, obj2Cam(cv::Rect(0, 0, 3, 3)));
            tvec.copyTo(obj2Cam(cv::Rect(3, 0, 1, 3)));
            
            roll = atan2(obj2Cam.at<double>(2,1), obj2Cam.at<double>(2,2));
            pitch = atan2(-obj2Cam.at<double>(2,0), sqrt(obj2Cam.at<double>(2,1)*obj2Cam.at<double>(2,1) + obj2Cam.at<double>(2,2)*obj2Cam.at<double>(2,2)));
            yaw = atan2(obj2Cam.at<double>(1,0), obj2Cam.at<double>(0,0));
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "solvePnP failed");
            return;
        }
    }

    void publish_tf(const cv::Mat& translation, double roll, double pitch, double yaw)
    {
        transformation.header.stamp = this->get_clock()->now();
        transformation.header.frame_id = "camera";
        transformation.child_frame_id = "target";
        
        transformation.transform.translation.x = translation.at<double>(0);
        transformation.transform.translation.y = translation.at<double>(1);
        transformation.transform.translation.z = translation.at<double>(2);
        
        q.setRPY(roll, pitch, yaw);
        transformation.transform.rotation = tf2::toMsg(q);
        
        broadcaster_->sendTransform(transformation);
    }

    void image_preprocess(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        framePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        frame = framePtr->image;
        
        if (frame.empty()) {
            RCLCPP_ERROR(get_logger(), "Received empty frame");
            return;
        }
        
        frame_copy = frame.clone();
        cv::cvtColor(frame, frame, cv::COLOR_BGR2HSV);
        cv::Scalar lowerHSV(hmin, smin, vmin);
        cv::Scalar upperHSV(hmax, smax, vmax);
        cv::inRange(frame, lowerHSV, upperHSV, frame);
        element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(13, 13));

        dilate(frame, frame, element);
        cv::findContours(frame, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        RCLCPP_INFO(get_logger(), "Found %lu contours", contours.size());

        conPoly.clear();
        triangle.clear();
        circle.clear();
        radius.clear();
        Points.clear();

        conPoly.resize(contours.size());
        triangle.resize(contours.size());
        circle.resize(contours.size());
        radius.resize(contours.size());

        for (size_t i = 0; i < contours.size(); i++)
        {   
            contourArea = cv::contourArea(contours[i]);
            if (contourArea > 400 && contourArea < 12000)
            {
                peri = cv::arcLength(contours[i], true);
                cv::approxPolyDP(contours[i], conPoly[i], 0.02*peri, true);
                
                if(conPoly[i].size() > 5 && conPoly[i].size() < 9)
                {
                    cv::minEnclosingTriangle(cv::Mat(conPoly[i]), triangle[i]);
                    cv::minEnclosingCircle(cv::Mat(contours[i]), circle[i], radius[i]);
                    if (conPoly.size() == 4)
                    {
                        center = (circle[0] + circle[1] + circle[2] + circle[3]) /4;
                        if (triangle[i].size() == 3) 
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
                                Points.push_back(cv::Point(triangle[i][0]));
                            }
                            else if (length1_square > length0_square && length1_square > length2_square)
                            {
                                Points.push_back(cv::Point(triangle[i][1]));
                            }
                            else if (length2_square > length0_square && length2_square > length1_square)
                            {
                                Points.push_back(cv::Point(triangle[i][2]));
                            }
                        }
                    }
                    
                }
            }
        }

        std::vector<double> length2O(Points.size());
        for (size_t i = 0; i < Points.size(); i++)
        {
            length2O[i] = Points[i].x * Points[i].x + Points[i].y * Points[i].y;
        }
        for (size_t j = length2O.size(); j > 0; j--)
        {
            for (size_t i = 0; i < j; i++)
            {
                if (length2O[i] < length2O[i+1])
                {
                    double temp = length2O[i];
                    length2O[i] = length2O[i+1];
                    length2O[i+1] = temp;
    
                    cv::Point ptemp = Points[i];
                    Points[i] = Points[i+1];
                    Points[i+1] = ptemp;
                }
            }
        }

        Points = filter(Points);


        for (const auto& point : Points)
        {
            if (point.x >= 0 && point.x < frame_copy.cols && point.y >= 0 && point.y < frame_copy.rows) {
                cv::circle(frame_copy, point, 5, cv::Scalar(0, 255, 0), cv::FILLED);
            }
        }
        for (const auto& point : circle)
        {
            if (point.x >= 0 && point.x < frame_copy.cols && point.y >= 0 && point.y < frame_copy.rows) {
                cv::circle(frame_copy, point, 5, cv::Scalar(0, 0, 255), cv::FILLED);
            }
        }
        cv::circle(frame_copy, center, 5, cv::Scalar(0,100,200), cv::FILLED);

        
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