#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <drone_interfaces/msg/sensor_data.hpp>
#include <string>
#include <sstream>
#include <cmath>
#include <unordered_map>
#include <memory>

using namespace std::chrono_literals;

class SerialReader : public rclcpp::Node
{
public:
    SerialReader()
    : Node("Sensor_node")
    {
        serial_port_ = "/dev/ttyUSB0"; 
        baud_rate_ = 115200;

        // Initialize the tag_maps for GPS and AHRS data
        sensor_tag_map = {
            {"ahrs", &SerialReader::parse_ahrs_data},
            {"gps", &SerialReader::parse_gps_data}
        };

        RCLCPP_INFO(this->get_logger(), "Starting Sensor node. Trying to connect to serial port %s.", serial_port_.c_str());
        sensor_publisher_ = this->create_publisher<drone_interfaces::msg::SensorData>("/sensor", 10);
        timer_ = this->create_wall_timer(5ms, std::bind(&SerialReader::read_sensor, this));
        
        try {
            serial_.setPort(serial_port_);
            serial_.setBaudrate(baud_rate_);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(timeout);
            serial_.open();
        } catch (serial::IOException &e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port %s", serial_port_.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "SerialReader node started, reading from %s at %d baud rate.",
                    serial_port_.c_str(), baud_rate_);
    }

private:
    std::unordered_map<std::string, double drone_interfaces::msg::SensorData::*> ahrs_tag_map = {
        {"acc_x", &drone_interfaces::msg::SensorData::acc_x},
        {"acc_y", &drone_interfaces::msg::SensorData::acc_y},
        {"acc_z", &drone_interfaces::msg::SensorData::acc_z},
        {"gyro_x", &drone_interfaces::msg::SensorData::gyro_x},
        {"gyro_y", &drone_interfaces::msg::SensorData::gyro_y},
        {"gyro_z", &drone_interfaces::msg::SensorData::gyro_z},
        {"mag_x", &drone_interfaces::msg::SensorData::mag_x},
        {"mag_y", &drone_interfaces::msg::SensorData::mag_y},
        {"mag_z", &drone_interfaces::msg::SensorData::mag_z},
        {"altitude", &drone_interfaces::msg::SensorData::altitude}
    };

    std::unordered_map<std::string, double drone_interfaces::msg::SensorData::*> gps_tag_map = {
        {"lat", &drone_interfaces::msg::SensorData::lat},
        {"lon", &drone_interfaces::msg::SensorData::lon},
        {"time", &drone_interfaces::msg::SensorData::time},
        {"speed", &drone_interfaces::msg::SensorData::speed},
        {"pdop", &drone_interfaces::msg::SensorData::pdop},
        {"hdop", &drone_interfaces::msg::SensorData::hdop}
    };

    std::unordered_map<std::string, std::shared_ptr<std::unordered_map<std::string, std::string>>(SerialReader::*)(const std::string&)> sensor_tag_map;

    void read_sensor()
    {
        if (serial_.available())
        {
            try {
                std::string line = serial_.readline();
                //RCLCPP_INFO(this->get_logger(), "Read from serial: %s", line.c_str());

                auto msg = drone_interfaces::msg::SensorData();
                bool new_data = false;

                for (const auto& tag : sensor_tag_map) {
                    // Check if the line contains the tag
                    if (line.find("<" + tag.first + ">") != std::string::npos && line.find("</" + tag.first + ">") != std::string::npos)
                    {
                        auto data = (this->*tag.second)(line);
                        if (data)
                        {
                            // Iterate through each tag and set the corresponding member in the msg object
                            for (const auto& tag : ahrs_tag_map) {
                                if (data->count(tag.first)) {
                                    msg.*(tag.second) = string_to_double(data->at(tag.first));
                                } else {
                                    msg.*(tag.second) = NAN;
                                }
                            }

                            new_data = true;
                        }
                        // If we found the tag, break the loop
                        break;
                    }
                }

                // Parse the GPS data from the string
                if (new_data)
                {
                    sensor_publisher_->publish(msg);
                }
            } catch (serial::IOException &e) {
                RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", e.what());
            }
        }
    }

    std::shared_ptr<std::unordered_map<std::string, std::string>> parse_sensor_data(
        const std::string& data, 
        const std::unordered_map<std::string, double drone_interfaces::msg::SensorData::*>& tag_map)
    {
        auto sensor_data = std::make_shared<std::unordered_map<std::string, std::string>>();

        auto parse_tag_value = [&data](const std::string& tag) -> std::string {
            std::string start_tag = "<" + tag + ">";
            std::string end_tag = "</" + tag + ">";
            size_t start_pos = data.find(start_tag);
            size_t end_pos = data.find(end_tag);
            if (start_pos != std::string::npos && end_pos != std::string::npos)
            {
                start_pos += start_tag.length();
                return data.substr(start_pos, end_pos - start_pos);
            }
            return "";
        };

        for (const auto& tag : tag_map) {
            (*sensor_data)[tag.first] = parse_tag_value(tag.first);
        }

        return sensor_data;
    }

    std::shared_ptr<std::unordered_map<std::string, std::string>> parse_gps_data(const std::string& data)
    {
        return parse_sensor_data(data, gps_tag_map);
    }

    std::shared_ptr<std::unordered_map<std::string, std::string>> parse_ahrs_data(const std::string& data)
    {
        return parse_sensor_data(data, ahrs_tag_map);
    }

    double string_to_double(const std::string& str)
    {
        try
        {
            return std::stod(str);
        }
        catch (...)
        {
            return NAN;
        }
    }

    rclcpp::Publisher<drone_interfaces::msg::SensorData>::SharedPtr sensor_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    serial::Serial serial_;
    std::string serial_port_;
    uint32_t baud_rate_;
};

int main(int argc, char *argv[])
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialReader>());
    rclcpp::shutdown();
    return 0;
}
