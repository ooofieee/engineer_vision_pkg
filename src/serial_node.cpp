#include "serial/serial.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include <chrono>

using namespace std::chrono_literals;

class serial_node : public rclcpp::Node
{
private:
    serial::Serial serial_port;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    geometry_msgs::msg::TransformStamped transformation;
    double roll, pitch, yaw;
    std::vector<double> xyz;
    double data[6] = {0};
public:
    serial_node(std::string &node_name) : Node(node_name)
    {
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, this, false);
        listen_tf();

        serial_set();
        std::vector<uint8_t> Txbuffer(sizeof(data));
        auto timer_ = this->create_wall_timer(200ms, [&]() -> void {
        data[0] = roll;
        data[1] = pitch;
        data[2] = yaw;
        data[3] = xyz[0];
        data[4] = xyz[1];
        data[5] = xyz[2];
        std::memcpy(Txbuffer.data(), data, sizeof(data));
        serial_port.write(Txbuffer);
        });
    }

    void listen_tf()
    {
        transformation = buffer_->lookupTransform("camera", "target", rclcpp::Time(0));
        auto rotation = transformation.transform.rotation;
        auto translation = transformation.transform.translation;
        try
        {
            tf2::getEulerYPR(rotation, yaw, pitch, roll);
            xyz[0] = translation.x;
            xyz[1] = translation.y;
            xyz[2] = translation.z;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "TransformException: %s", ex.what());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "rotation: %f, %f, %f", roll, pitch, yaw);
    }

    void serial_set()
    {
        serial_port.setPort("/dev/ttyUSB0");
        serial_port.setBaudrate(115200);
        serial::Timeout time_ = serial::Timeout::simpleTimeout(2000);
        serial_port.setTimeout(time_);
        serial_port.open();
        if (serial_port.isOpen())
        {
            RCLCPP_INFO(this->get_logger(), "serial port opened");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "serial port error");
        }
    }

};

int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string node_name = "serial_node";
    auto node = std::make_shared<serial_node>(node_name);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}