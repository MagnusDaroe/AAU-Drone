#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <drone/msg/gps_data.hpp>
#include <string>
#include <sstream>
#include <cmath>
#include <unordered_map>
#include <memory>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>

using namespace std::chrono_literals;
using namespace boost::asio;

class SerialReader : public rclcpp::Node
{
public:
    SerialReader()
    : Node("GPS_node"), io_(), serial_(io_, serial_port_)
    {
        // Initialize the publisher
        publisher_ = this->create_publisher<drone::msg::GPSData>("gps_data", 10);

        // Open the serial port
        serial_.set_option(serial_port_base::baud_rate(baud_rate_));
        
        RCLCPP_INFO(this->get_logger(), "SerialReader node started, reading from %s at %d baud rate.",
                    serial_port_.c_str(), baud_rate_);
        
        // Start an asynchronous read operation
        start_async_read();
        
        // Spin the IO service in a separate thread
        io_thread_ = std::thread([this]() { io_.run(); });
    }

    ~SerialReader()
    {
        // Stop the IO service and join the thread
        io_.stop();
        if (io_thread_.joinable()) {
            io_thread_.join();
        }
    }

private:
    void start_async_read()
    {
        async_read_until(serial_, buffer_, '\n',
            boost::bind(&SerialReader::handle_read, this,
            placeholders::error, boost::placeholders::_2));
    }

    void handle_read(const boost::system::error_code& error, size_t /*bytes_transferred*/)
    {
        if (!error)
        {
            std::istream is(&buffer_);
            std::string line;
            std::getline(is, line);
            RCLCPP_INFO(this->get_logger(), "Read from serial: %s", line.c_str());

            // Parse the GPS data from the string
            if (line.find("<gps>") != std::string::npos && line.find("</gps>") != std::string::npos)
            {
                auto data = parse_gps_data(line);
                if (data)
                {
                    auto msg = drone::msg::GPSData();
                    msg.lat = string_to_double(data->at("lat"));
                    msg.lon = string_to_double(data->at("lon"));
                    msg.date = data->at("date"); // Assuming date is a string
                    msg.time = data->at("time"); // Assuming time is a string
                    msg.course = string_to_double(data->at("course"));
                    msg.speed = string_to_double(data->at("speed"));
                    msg.pdop = string_to_double(data->at("pdop"));
                    msg.hdop = string_to_double(data->at("hdop"));
                    publisher_->publish(msg);
                }
            }

            // Start another asynchronous read operation
            start_async_read();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", error.message().c_str());
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
        (*gps_data)["date"] = parse_tag_value("date");
        (*gps_data)["time"] = parse_tag_value("time");
        (*gps_data)["course"] = parse_tag_value("course");
        (*gps_data)["speed"] = parse_tag_value("speed");
        (*gps_data)["pdop"] = parse_tag_value("pdop");
        (*gps_data)["hdop"] = parse_tag_value("hdop");

        return gps_data;
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

    rclcpp::Publisher<drone::msg::GPSData>::SharedPtr publisher_;
    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    boost::asio::streambuf buffer_;
    std::thread io_thread_;
    std::string serial_port_ = "/dev/ttyUSB0";
    uint32_t baud_rate_ = 115200;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialReader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
