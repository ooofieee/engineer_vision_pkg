#include "serial/serial.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

using namespace std::chrono_literals;

class serial_test_node : public rclcpp::Node
{
private:
    serial::Serial serial_port;
    rclcpp::TimerBase::SharedPtr timer_;
    unsigned char Txbuffer[1] = {0};
    unsigned char Rxbuffer[5] = {0};

public:
    explicit serial_test_node(const std::string &node_name) : Node(node_name)
    {
        serial_port.setPort("/dev/ttyUSB0");
        serial_port.setBaudrate(115200);
        serial::Timeout time_ = serial::Timeout::simpleTimeout(2000);
        serial_port.setTimeout(time_);
        serial_port.open();
        if (serial_port.isOpen()){RCLCPP_INFO(this->get_logger(), "serial port opened");}
        else {RCLCPP_ERROR(this->get_logger(), "serial port error");}
        timer_ = this->create_wall_timer(1000ms, [&]() -> void {
            serial_port.read(Rxbuffer, 5);
            RCLCPP_INFO(this->get_logger(), "Rxbuffer: %s", Rxbuffer);
            Txbuffer[0] = '1';
            serial_port.write(Txbuffer, 1);
        });

    }

};

int main (int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<serial_test_node>("serial_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
}