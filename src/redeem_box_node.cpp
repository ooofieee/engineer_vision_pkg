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

class redeem_box_node : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    cv_bridge::CvImagePtr framePtr;
    cv::Mat frame, dil_kernel, ero_kernel, frame_preprocessed, frame_copy;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    int hmin = 83, smin = 49, vmin = 213;
    int hmax = 179, smax = 255, vmax = 245;
    double contourArea;
    float peri;
    std::vector<std::vector<cv::Point>> conPoly;
    std::vector<std::vector<cv::Point2f>> triangle;
    std::vector<cv::Point2f> circle, circle_pt_sorted, Points_float, triangle_pt_sorted, point_temp;
    std::vector<float> radius, Point2O;
    std::vector<int> index;
    std::vector<cv::Point> Points;
    std::queue<std::vector<cv::Point2f>> point_queue_circle, point_queue_triangle;
    cv::Point2f center;
    cv::Point max_corner;
    cv::Mat camera_matrix, distortion_coefficients, rectification_matrix, projection_matrix, rvec, tvec;
    std::vector<cv::Point3f> objPoints = {cv::Point3f(0.0f, 0.0f, 0.0f),
                                          cv::Point3f(240.0f, 0.0f, 0.0f),
                                          cv::Point3f(240.0f,240.0f, 0.0f),
                                          cv::Point3f(0.0f,240.0f, 0.0f)};
    cv::Mat obj2Cam = cv::Mat::zeros(3, 3, CV_64FC1);
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
        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("redeem_box_image", 10, [&](const sensor_msgs::msg::Image::SharedPtr msg) -> void
                                                                         {
                                                                             image_preprocess(msg);
                                                                             targeting();
                                                                             pnpSolver();
                                                                             //publish_tf(tvec, roll, pitch, yaw);
                                                                         });
    }

    void loadCameraParams()
    {
        cv::FileStorage fs("/home/mac/Code/ws_0/src/engineer_vision_pkg/config/camera_info.yaml", cv::FileStorage::READ);
        fs["camera_matrix"] >> camera_matrix;
        fs["distortion_coefficients"] >> distortion_coefficients;
        fs["rectification_matrix"] >> rectification_matrix;
        fs["projection_matrix"] >> projection_matrix;
        fs.release();
    }

    std::vector<cv::Point2f> filter(std::vector<cv::Point2f> &points, std::queue<std::vector<cv::Point2f>> &point_queue)
    {
        point_temp.resize(4, cv::Point2f(0.0, 0.0));

        auto isZeroPoint = [](const cv::Point2f &pt)
        {
            return pt.x == 0.0f && pt.y == 0.0f;
        };

        if (points.size() != 4 || std::any_of(points.begin(), points.end(), isZeroPoint))
        {
            RCLCPP_WARN(this->get_logger(), "Invalid or zero points in input, skipping filter");
            return point_temp;
        }

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
            const auto &newest = point_queue.back();
            const auto &oldest = point_queue.front();

            if (newest.size() == 4 && oldest.size() == 4 &&
                !std::any_of(newest.begin(), newest.end(), isZeroPoint) &&
                !std::any_of(oldest.begin(), oldest.end(), isZeroPoint))
            {
                for (size_t i = 0; i < 4; i++)
                {
                    point_temp[i] = newest[i] * 0.2f + oldest[i] * 0.8f;
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Queue contains zero points, skipping filter");
            }
            return point_temp;
        }

        return point_temp;
    }

    std::vector<cv::Point2f> sortPointsClockwise(const std::vector<cv::Point2f> &points, const cv::Point2f &center)
    {
        std::vector<std::pair<cv::Point2f, double>> angle_point_pairs;

        for (const auto &pt : points)
        {
            double angle = atan2(pt.y - center.y, pt.x - center.x);
            angle_point_pairs.emplace_back(pt, angle);
        }

        std::sort(angle_point_pairs.begin(), angle_point_pairs.end(),
                  [](const std::pair<cv::Point2f, double> &a, const std::pair<cv::Point2f, double> &b)
                  {
                      return a.second > b.second;
                  });

        std::vector<cv::Point2f> sorted;
        for (const auto &pair : angle_point_pairs)
        {
            sorted.push_back(pair.first);
        }

        return sorted;
    }

    bool DistVarianceValidation(const std::vector<cv::Point2f> &circle_pts, const std::vector<cv::Point2f> &triangle_pts, float max_variance = 100.0f)
    {
        if (circle_pts.size() != 4 || triangle_pts.size() != 4)
            return false;

        std::vector<float> dists;
        for (int i = 0; i < 4; ++i)
        {
            float d = cv::norm(circle_pts[i] - triangle_pts[i]);
            dists.push_back(d);
        }

        float mean = std::accumulate(dists.begin(), dists.end(), 0.0f) / dists.size();

        float variance = 0.0f;
        for (float d : dists)
        {
            variance += (d - mean) * (d - mean);
        }
        variance /= dists.size();

        RCLCPP_INFO(rclcpp::get_logger("validate"), "circle↔triangle dist variance: %.2f", variance);

        return variance < max_variance;
    }

    void pnpSolver()
    {
        if(circle_pt_sorted.size() != 4 || triangle_pt_sorted.size() != 4)
        {
            RCLCPP_WARN(get_logger(), "Invalid number of points for PnP");
            return;
        }
        bool succees = cv::solvePnP(objPoints, circle_pt_sorted, camera_matrix, distortion_coefficients, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
        if (succees)
        {
            Rodrigues(rvec, obj2Cam);
            obj2Cam = obj2Cam.t();
            roll = atan2(obj2Cam.at<double>(2, 1), obj2Cam.at<double>(2, 2));
            pitch = atan2(-obj2Cam.at<double>(2, 0), sqrt(obj2Cam.at<double>(2, 1) * obj2Cam.at<double>(2, 1) + obj2Cam.at<double>(2, 2) * obj2Cam.at<double>(2, 2)));
            yaw = atan2(obj2Cam.at<double>(1, 0), obj2Cam.at<double>(0, 0));
            RCLCPP_INFO(get_logger(), "tvec: (%f, %f, %f)", tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
            RCLCPP_INFO(get_logger(), "rvec: (%f, %f, %f)", roll, pitch, yaw);
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "solvePnP failed");
            return;
        }
    }

    void publish_tf(const cv::Mat &translation, double roll, double pitch, double yaw)
    {
        if (!(translation.rows == 3 && translation.cols == 1) &&
            !(translation.rows == 1 && translation.cols == 3))
        {
            RCLCPP_WARN(this->get_logger(), "Invalid translation shape: expected 3x1 or 1x3, got %dx%d", translation.rows, translation.cols);
            return;
        }
    
        if (translation.type() != CV_64F)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid translation type: expected CV_64F");
            return;
        }
    
        if (!std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw))
        {
            RCLCPP_WARN(this->get_logger(), "Invalid rotation angles: roll=%f, pitch=%f, yaw=%f", roll, pitch, yaw);
            return;
        }
    
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

        if (frame.empty())
        {
            RCLCPP_ERROR(get_logger(), "Received empty frame");
            return;
        }

        frame_copy = frame.clone();
        cv::cvtColor(frame, frame, cv::COLOR_BGR2HSV);
        cv::Scalar lowerHSV(hmin, smin, vmin);
        cv::Scalar upperHSV(hmax, smax, vmax);
        cv::inRange(frame, lowerHSV, upperHSV, frame);
        dil_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(13, 13));
        ero_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
        // cv::erode(frame, frame, ero_kernel);
        cv::dilate(frame, frame, dil_kernel);
        cv::morphologyEx(frame, frame, cv::MORPH_CLOSE, dil_kernel);
        cv::findContours(frame, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::drawContours(frame, contours, -1, cv::Scalar(255), cv::FILLED);
        //RCLCPP_INFO(get_logger(), "%lu contours found", contours.size());
    }

    void targeting()
    {
        conPoly.clear();
        triangle.clear();
        circle.clear();
        radius.clear();
        Points.clear();
        index.clear();
        conPoly.clear();
        circle_pt_sorted.clear();
        triangle_pt_sorted.clear();
        Points_float.clear();

        conPoly.resize(contours.size());
        triangle.resize(contours.size());
        circle.resize(contours.size());
        radius.resize(contours.size());

        for (size_t i = 0; i < contours.size(); i++)
        {
            contourArea = cv::contourArea(contours[i]);
            if (contourArea > 500 && contourArea < 3000)
            {
                peri = cv::arcLength(contours[i], true);
                cv::approxPolyDP(contours[i], conPoly[i], 0.02 * peri, true);

                if (conPoly[i].size() > 5 && conPoly[i].size() < 9)
                {
                    cv::minEnclosingCircle(cv::Mat(conPoly[i]), circle[i], radius[i]);
                    index.push_back(i);
                    cv::minEnclosingTriangle(contours[i], triangle[i]);
                }
            }
        }
        //RCLCPP_INFO(get_logger(), "%lu targets found", index.size());

        if (index.size() == 4)
        {
            center = (circle[index[0]] + circle[index[1]] + circle[index[2]] + circle[index[3]]) / 4;

            //RCLCPP_INFO(get_logger(), "center: (%f, %f)", center.x, center.y);

            for (int i = 0; i < std::min(4, static_cast<int>(index.size())); i++)
            {
                if (triangle[index[i]].size() == 3)
                {
                    double length0_square = (triangle[index[i]][1].x - triangle[index[i]][2].x) * (triangle[index[i]][1].x - triangle[index[i]][2].x) +
                                            (triangle[index[i]][1].y - triangle[index[i]][2].y) * (triangle[index[i]][1].y - triangle[index[i]][2].y);
                    double length1_square = (triangle[index[i]][0].x - triangle[index[i]][2].x) * (triangle[index[i]][0].x - triangle[index[i]][2].x) +
                                            (triangle[index[i]][0].y - triangle[index[i]][2].y) * (triangle[index[i]][0].y - triangle[index[i]][2].y);
                    double length2_square = (triangle[index[i]][1].x - triangle[index[i]][0].x) * (triangle[index[i]][1].x - triangle[index[i]][0].x) +
                                            (triangle[index[i]][1].y - triangle[index[i]][0].y) * (triangle[index[i]][1].y - triangle[index[i]][0].y);

                    if (length0_square > length1_square && length0_square > length2_square)
                        max_corner = triangle[index[i]][0];
                    else if (length1_square > length0_square && length1_square > length2_square)
                        max_corner = triangle[index[i]][1];
                    else
                        max_corner = triangle[index[i]][2];

                    Points_float.emplace_back(static_cast<cv::Point2f>(max_corner));
                }
            }

            circle_pt_sorted = sortPointsClockwise({circle[index[0]], circle[index[1]], circle[index[2]], circle[index[3]]}, center);
            triangle_pt_sorted = sortPointsClockwise(Points_float, center);

            //RCLCPP_INFO(this->get_logger(), "triangle_pt_sorted num: %lu", triangle_pt_sorted.size());
            //RCLCPP_INFO(this->get_logger(), "circle_pt_sorted num: %lu", circle_pt_sorted.size());

            circle_pt_sorted = filter(circle_pt_sorted, point_queue_circle);
            triangle_pt_sorted = filter(triangle_pt_sorted, point_queue_triangle);

            if(!DistVarianceValidation(circle_pt_sorted, triangle_pt_sorted))
            {
                //RCLCPP_WARN(this->get_logger(), "circle and triangle points variance too large");
                return;
            }

            for (const auto &point : triangle_pt_sorted)
            {
                if (point.x >= 0 && point.x < frame_copy.cols && point.y >= 0 && point.y < frame_copy.rows)
                {
                    cv::circle(frame_copy, point, 5, cv::Scalar(0, 255, 0), cv::FILLED);
                }
            }
            for (const auto &point : circle_pt_sorted)
            {
                if (point.x > 0 && point.x < frame_copy.cols && point.y > 0 && point.y < frame_copy.rows)
                {
                    cv::circle(frame_copy, point, 5, cv::Scalar(0, 0, 255), cv::FILLED);
                }
            }

            cv::line(frame_copy, circle_pt_sorted[0], circle_pt_sorted[1], cv::Scalar(255, 0, 0), 2);
            cv::line(frame_copy, circle_pt_sorted[1], circle_pt_sorted[2], cv::Scalar(255, 0, 0), 2);
            cv::line(frame_copy, circle_pt_sorted[2], circle_pt_sorted[3], cv::Scalar(255, 0, 0), 2);
            cv::line(frame_copy, circle_pt_sorted[3], circle_pt_sorted[0], cv::Scalar(255, 0, 0), 2);
            cv::line(frame_copy, circle_pt_sorted[0], triangle_pt_sorted[0], cv::Scalar(0, 255, 0), 2);
            cv::line(frame_copy, circle_pt_sorted[1], triangle_pt_sorted[1], cv::Scalar(0, 255, 0), 2);
            cv::line(frame_copy, circle_pt_sorted[2], triangle_pt_sorted[2], cv::Scalar(0, 255, 0), 2);
            cv::line(frame_copy, circle_pt_sorted[3], triangle_pt_sorted[3], cv::Scalar(0, 255, 0), 2);
            cv::line(frame_copy, triangle_pt_sorted[0], triangle_pt_sorted[1], cv::Scalar(0, 0, 255), 2);
            cv::line(frame_copy, triangle_pt_sorted[1], triangle_pt_sorted[2], cv::Scalar(0, 0, 255), 2);
            cv::line(frame_copy, triangle_pt_sorted[2], triangle_pt_sorted[3], cv::Scalar(0, 0, 255), 2);
            cv::line(frame_copy, triangle_pt_sorted[3], triangle_pt_sorted[0], cv::Scalar(0, 0, 255), 2);

            cv::circle(frame_copy, center, 5, cv::Scalar(0, 100, 200), cv::FILLED);
            cv::putText(frame_copy, "0", circle_pt_sorted[0], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
            cv::putText(frame_copy, "1", circle_pt_sorted[1], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 155, 255), 2);
            cv::putText(frame_copy, "2", circle_pt_sorted[2], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 155, 255), 2);
            cv::putText(frame_copy, "3", circle_pt_sorted[3], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 155, 255), 2);
            cv::putText(frame_copy, "center", center, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
            cv::putText(frame_copy, "0", triangle_pt_sorted[0], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
            cv::putText(frame_copy, "1", triangle_pt_sorted[1], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 155, 255), 2);
            cv::putText(frame_copy, "2", triangle_pt_sorted[2], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 155, 255), 2);
            cv::putText(frame_copy, "3", triangle_pt_sorted[3], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 155, 255), 2);
        }
        auto msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_copy).toImageMsg();
        publisher_->publish(*msg_);
    }

    // for (size_t i = 0;i< index.size(); i++)
    // {
    //     if (circle[index[i]] == cv::Point2f(0.0,0.0))
    //     {
    //         circle[index[i]] =
    //     }
    // }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<redeem_box_node>("redeem_box_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
