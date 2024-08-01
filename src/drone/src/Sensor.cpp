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
    : Node("Sensor_node"),
      ahrs_tag_map(initialize_ahrs_tag_map()),
      gps_tag_map(initialize_gps_tag_map())
    {
        serial_port_ = "/dev/ttyUSB0"; 
        baud_rate_ = 115200;

        // Initializing sensor_tag_map with pointers to member functions
        sensor_tag_map = {
            {"AHRS", &SerialReader::parse_ahrs_data},
            {"GPS", &SerialReader::parse_gps_data}
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
    // Maps to associate sensor data tags with member variable pointers
    std::unordered_map<std::string, double drone_interfaces::msg::SensorData::*> ahrs_tag_map;
    std::unordered_map<std::string, double drone_interfaces::msg::SensorData::*> gps_tag_map;
    std::unordered_map<std::string, std::shared_ptr<std::unordered_map<std::string, std::string>>(SerialReader::*)(const std::string&)> sensor_tag_map;

    // Static member function to initialize AHRS tag map
    static std::unordered_map<std::string, double drone_interfaces::msg::SensorData::*> initialize_ahrs_tag_map() {
        std::unordered_map<std::string, double drone_interfaces::msg::SensorData::*> map;
        map["ax"] = &drone_interfaces::msg::SensorData::acc_x;
        map["ay"] = &drone_interfaces::msg::SensorData::acc_y;
        map["az"] = &drone_interfaces::msg::SensorData::acc_z;
        map["gx"] = &drone_interfaces::msg::SensorData::gyro_x;
        map["gy"] = &drone_interfaces::msg::SensorData::gyro_y;
        map["gz"] = &drone_interfaces::msg::SensorData::gyro_z;
        map["mx"] = &drone_interfaces::msg::SensorData::mag_x;
        map["my"] = &drone_interfaces::msg::SensorData::mag_y;
        map["mz"] = &drone_interfaces::msg::SensorData::mag_z;
        //map["mh"] = &drone_interfaces::msg::SensorData::mag_h;
        map["alt"] = &drone_interfaces::msg::SensorData::altitude;
        return map;
    }

    // Static member function to initialize GPS tag map
    static std::unordered_map<std::string, double drone_interfaces::msg::SensorData::*> initialize_gps_tag_map() {
        std::unordered_map<std::string, double drone_interfaces::msg::SensorData::*> map;
        map["lat"] = &drone_interfaces::msg::SensorData::lat;
        map["lon"] = &drone_interfaces::msg::SensorData::lon;
        //map["time"] = &drone_interfaces::msg::SensorData::time;
        map["v"] = &drone_interfaces::msg::SensorData::speed;
        map["p"] = &drone_interfaces::msg::SensorData::pdop;
        map["h"] = &drone_interfaces::msg::SensorData::hdop;
        return map;
    }

    // Function to read sensor data from the serial port

    void read_sensor() {
        if (serial_.available()) {
            try {
                std::string line = serial_.readline();
                //RCLCPP_INFO(this->get_logger(), "Read from serial: %s", line.c_str());

                auto msg = drone_interfaces::msg::SensorData();
                bool new_data = false;

                //Set all values to NAN
                for (const auto& tag : ahrs_tag_map) {
                    msg.*(tag.second) = NAN;
                }

                for (const auto& tag : gps_tag_map) {
                    msg.*(tag.second) = NAN;
                }

                // Determine if the line contains AHRS or GPS data
                std::unordered_map<std::string, double drone_interfaces::msg::SensorData::*>* tag_map = nullptr;

                if (line.find("<AHRS>") != std::string::npos) {
                    tag_map = &ahrs_tag_map;
                } else if (line.find("<GPS>") != std::string::npos) {
                    tag_map = &gps_tag_map;
                }
                else {
                    //Information about the sensor setup, which is not parseable:
                    RCLCPP_INFO(this->get_logger(), "Sensor Setup: %s", line.c_str());
                    
                    //Make tag_map be false
                    tag_map = nullptr;
                }

                if (tag_map) {
                    auto data = parse_sensor_data(line, *tag_map);

                    if (data) {
                        // Iterate through each tag and set the corresponding member in the msg object
                        for (const auto& tag : *tag_map) {
                            const std::string& field = tag.first;
                            if (data->count(field)) {
                                msg.*(tag.second) = string_to_double(data->at(field));
                            } else {
                                msg.*(tag.second) = NAN;
                            }
                        }

                        new_data = true;
                    }
                }

                if (new_data) {
                    sensor_publisher_->publish(msg);
                }
            } catch (serial::IOException &e) {
                RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", e.what());
            }
        }
    }


    // Function to parse sensor data using the provided tag map
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

        //First find the tag, then find the nested tags and parse them


        for (const auto& tag : tag_map) {
            (*sensor_data)[tag.first] = parse_tag_value(tag.first);
        }

        return sensor_data;
    }

    // Function to parse GPS data
    std::shared_ptr<std::unordered_map<std::string, std::string>> parse_gps_data(const std::string& data)
    {
        return parse_sensor_data(data, gps_tag_map);
    }

    // Function to parse AHRS data
    std::shared_ptr<std::unordered_map<std::string, std::string>> parse_ahrs_data(const std::string& data)
    {
        return parse_sensor_data(data, ahrs_tag_map);
    }

    // Function to convert string to double
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
