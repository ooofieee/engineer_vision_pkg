#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv4/opencv2/imgcodecs.hpp"
#include "opencv4/opencv2/imgproc.hpp"
#include "opencv4/opencv2/highgui.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv4/opencv2/dnn.hpp"

class yolo_node : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    cv_bridge::CvImagePtr framePtr;
    cv::Mat frame, frame_resized, output;
    cv::dnn::Net model;
    std::vector<cv::Rect> boxes;
    const int input_width = 640;
    const int input_height = 640;

public:
    explicit yolo_node(const std::string node_name) : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "YOLO node launched");
        loadOnnxModel("/home/ooofieee/Code/ws_0/src/engineer_vision_pkg/resource/best.onnx");
        RCLCPP_INFO(this->get_logger(), "Model loaded successfully");
        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("redeem_box_image", 10, [&](const sensor_msgs::msg::Image::SharedPtr msg) ->void {
            framePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            frame = framePtr->image;
            RCLCPP_INFO(this->get_logger(), "Frame received");
            preProcess();
            postProcess();
            for (size_t i = 0; i < boxes.size(); i++)
            {
                RCLCPP_INFO(this->get_logger(), "Box %ld : %d, %d, %d, %d", i, boxes[i].x, boxes[i].y, boxes[i].width, boxes[i].height);
            }
            std::cout<<std::endl;
        });
    }

    void loadOnnxModel(const std::string model_path)
    {
        model = cv::dnn::readNetFromONNX(model_path);
        if (model.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load model from %s", model_path.c_str());
            return;
        }
    }

    void preProcess()
    {
        try 
        {
            cv::resize(frame, frame_resized, cv::Size(input_width, input_height));
            cv::Mat blob = cv::dnn::blobFromImage(frame_resized, 1.0/255.0, cv::Size(input_width, input_height), cv::Scalar(0, 0, 0), true, false);
            model.setInput(blob);
            if (model.empty()) 
            {
                RCLCPP_ERROR(this->get_logger(), "Model is empty, cannot process frame");
                return;
            }
            output = model.forward();
            RCLCPP_INFO(this->get_logger(), "Preprocessing completed. Output shape: %d x %d", output.rows, output.cols);
        } 
        catch (const cv::Exception& e) 
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV error during preprocessing: %s", e.what());
        }
    }

    void postProcess(float confThreshold = 0.25)
    {
        boxes.clear();
        try {
            for (int i = 0; i < output.rows; ++i) 
            {
                float confidence = output.at<float>(i, 4);
                if (confidence > confThreshold) 
                {
                    float x = output.at<float>(i, 0);
                    float y = output.at<float>(i, 1);
                    float w = output.at<float>(i, 2);
                    float h = output.at<float>(i, 3);

                    x = x * frame.cols / input_width;
                    y = y * frame.rows / input_height;
                    w = w * frame.cols / input_width;
                    h = h * frame.rows / input_height;
                    
                    boxes.push_back(cv::Rect(x - w/2, y - h/2, w, h));
                }
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV error during postprocessing: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<yolo_node>("yolo_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}