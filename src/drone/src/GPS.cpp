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

        RCLCPP_INFO(this->get_logger(), "Starting Sensor node. Trying to connect to serial port %s.", serial_port_.c_str());
        sensor_publisher_ = this->create_publisher<drone_interfaces::msg::SensorData>("/sensor", 10);
        timer_ = this->create_wall_timer(5ms, std::bind(&SerialReader::timer_callback, this));
        
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
    void timer_callback()
    {
        if (serial_.available())
        {
            try {
                std::string line = serial_.readline();
                RCLCPP_INFO(this->get_logger(), "Read from serial: %s", line.c_str());

                auto msg = drone_interfaces::msg::SensorData();
                bool new_data = false;

                // Parse the GPS data from the string
                if (line.find("<gps>") != std::string::npos && line.find("</gps>") != std::string::npos)
                {
                    auto data = parse_gps_data(line);
                    if (data)
                    {
                        msg.lat = string_to_double(data->at("lat"));
                        msg.lon = string_to_double(data->at("lon"));
                        msg.time = data->at("time");
                        msg.speed = string_to_double(data->at("speed"));
                        msg.pdop = string_to_double(data->at("pdop"));
                        msg.hdop = string_to_double(data->at("hdop"));
                        new_data = true;
                    }
                }

                // Parse the AHRS data from the string
                if (line.find("<AHRS>") != std::string::npos && line.find("</AHRS>") != std::string::npos)
                {
                    auto data = parse_ahrs_data(line);
                    if (data)
                    {
                        if (data->count("acc_x")) msg.acc_x = string_to_double(data->at("acc_x"));
                        if (data->count("acc_y")) msg.acc_y = string_to_double(data->at("acc_y"));
                        if (data->count("acc_z")) msg.acc_z = string_to_double(data->at("acc_z"));
                        if (data->count("gyro_x")) msg.gyro_x = string_to_double(data->at("gyro_x"));
                        if (data->count("gyro_y")) msg.gyro_y = string_to_double(data->at("gyro_y"));
                        if (data->count("gyro_z")) msg.gyro_z = string_to_double(data->at("gyro_z"));
                        if (data->count("mag_x")) msg.mag_x = string_to_double(data->at("mag_x"));
                        if (data->count("mag_y")) msg.mag_y = string_to_double(data->at("mag_y"));
                        if (data->count("mag_z")) msg.mag_z = string_to_double(data->at("mag_z"));
                        if (data->count("altitude")) msg.altitude = string_to_double(data->at("altitude"));
                        new_data = true;
                    }
                }

                if (new_data)
                {
                    sensor_publisher_->publish(msg);
                }
            } catch (serial::IOException &e) {
                RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", e.what());
            }
        }
    }

    std::shared_ptr<std::unordered_map<std::string, std::string>> parse_gps_data(const std::string& data)
    {
        auto gps_data = std::make_shared<std::unordered_map<std::string, std::string>>();

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

        (*gps_data)["lat"] = parse_tag_value("lat");
        (*gps_data)["lon"] = parse_tag_value("lon");
        (*gps_data)["time"] = parse_tag_value("time");
        (*gps_data)["speed"] = parse_tag_value("speed");
        (*gps_data)["pdop"] = parse_tag_value("pdop");
        (*gps_data)["hdop"] = parse_tag_value("hdop");

        return gps_data;
    }

    std::shared_ptr<std::unordered_map<std::string, std::string>> parse_ahrs_data(const std::string& data)
    {
        auto ahrs_data = std::make_shared<std::unordered_map<std::string, std::string>>();

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

        (*ahrs_data)["acc_x"] = parse_tag_value("x");
        (*ahrs_data)["acc_y"] = parse_tag_value("y");
        (*ahrs_data)["acc_z"] = parse_tag_value("z");
        (*ahrs_data)["gyro_x"] = parse_tag_value("x");
        (*ahrs_data)["gyro_y"] = parse_tag_value("y");
        (*ahrs_data)["gyro_z"] = parse_tag_value("z");
        (*ahrs_data)["mag_x"] = parse_tag_value("x");
        (*ahrs_data)["mag_y"] = parse_tag_value("y");
        (*ahrs_data)["mag_z"] = parse_tag_value("z");
        (*ahrs_data)["altitude"] = parse_tag_value("altitude");

        return ahrs_data;
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
