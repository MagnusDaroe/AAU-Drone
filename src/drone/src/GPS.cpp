#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <serial/serial.h> 

class SerialReader : public rclcpp::Node
{
public:
    SerialReader()
    : Node("serial_reader"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("esp32_data", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SerialReader::timer_callback, this));

        // Adjust to your serial port and baud rate
        serial_port_ = "/dev/ttyUSB0";
        baud_rate_ = 115200;

        try
        {
            serial_ = new serial::Serial(serial_port_, baud_rate_, serial::Timeout::simpleTimeout(1000));
            RCLCPP_INFO(this->get_logger(), "SerialReader node started, reading from %s at %d baud rate.", serial_port_.c_str(), baud_rate_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error opening serial port: %s", e.what());
        }
    }

private:
    void timer_callback()
    {
        if (serial_->available() > 0)
        {
            std_msgs::msg::String msg;
            msg.data = serial_->readline();
            RCLCPP_INFO(this->get_logger(), "Read from serial: %s", msg.data.c_str());
            publisher_->publish(msg);
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string serial_port_;
    int baud_rate_;
    serial::Serial *serial_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialReader>());
    rclcpp::shutdown();
    return 0;
}
