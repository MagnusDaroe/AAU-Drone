#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <drone/msg/gps_data.hpp>
#include <string>
#include <sstream>
#include <cmath>

using namespace std::chrono_literals;

class SerialReader : public rclcpp::Node
{
public:
    SerialReader()
    : Node("serial_reader")
    {
        publisher_ = this->create_publisher<drone::msg::GPSData>("gps_data", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&SerialReader::timer_callback, this));
        
        serial_port_ = "/dev/ttyUSB0";  // Adjust to your serial port
        baud_rate_ = 115200;  // Adjust to your baud rate

        try {
            serial_.setPort(serial_port_);
            serial_.setBaudrate(baud_rate_);
            serial_.setTimeout(serial::Timeout::simpleTimeout(1000));
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

                // Parse the GPS data from the string
                if (line.find("<gps>") != std::string::npos && line.find("</gps>") != std::string::npos)
                {
                    auto data = parse_gps_data(line);
                    if (data)
                    {
                        auto msg = drone::msg::GPSData();
                        msg.lat = data->at("lat");
                        msg.lon = data->at("lon");
                        msg.date = data->at("date");
                        msg.time = data->at("time");
                        msg.course = data->at("course");
                        msg.speed = data->at("speed");
                        msg.pdop = data->at("pdop");
                        msg.hdop = data->at("hdop");
                        publisher_->publish(msg);
                    }
                }
            } catch (serial::IOException &e) {
                RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", e.what());
            }
        }
    }

    std::shared_ptr<std::unordered_map<std::string, float>> parse_gps_data(const std::string& data)
    {
        auto gps_data = std::make_shared<std::unordered_map<std::string, float>>();

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

        try
        {
            (*gps_data)["lat"] = std::stof(parse_tag_value("lat"));
        }
        catch (...)
        {
            (*gps_data)["lat"] = NAN;
        }

        try
        {
            (*gps_data)["lon"] = std::stof(parse_tag_value("lon"));
        }
        catch (...)
        {
            (*gps_data)["lon"] = NAN;
        }

        (*gps_data)["date"] = parse_tag_value("date");
        (*gps_data)["time"] = parse_tag_value("time");

        try
        {
            (*gps_data)["course"] = std::stof(parse_tag_value("course"));
        }
        catch (...)
        {
            (*gps_data)["course"] = NAN;
        }

        try
        {
            (*gps_data)["speed"] = std::stof(parse_tag_value("speed"));
        }
        catch (...)
        {
            (*gps_data)["speed"] = NAN;
        }

        try
        {
            (*gps_data)["pdop"] = std::stof(parse_tag_value("pdop"));
        }
        catch (...)
        {
            (*gps_data)["pdop"] = NAN;
        }

        try
        {
            (*gps_data)["hdop"] = std::stof(parse_tag_value("hdop"));
        }
        catch (...)
        {
            (*gps_data)["hdop"] = NAN;
        }

        return gps_data;
    }

    rclcpp::Publisher<drone::msg::GPSData>::SharedPtr publisher_;
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
