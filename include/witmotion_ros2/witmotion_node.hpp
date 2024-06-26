#ifndef WIT_TTL_ROS2__WITMOTION_NODE_HPP_
#define WIT_TTL_ROS2__WITMOTION_NODE_HPP_

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <asio.hpp>
#include <thread>
#include <vector>
#include "witmotion_ros2/witmotion_handler.hpp"

class WITMotionNode : public rclcpp::Node
{
public:
    WITMotionNode();

    void stopAsyncThread();

private:
    void handleSerialData(uint8_t raw_data);
    void publisher();
    void callback(const std_msgs::msg::String::SharedPtr data);

    rclcpp::Logger getLogger();
    void asyncThread();
    bool startAsyncThread();

    asio::io_context io_;
    asio::serial_port serial_port_;
    std::atomic<bool> stop_async_thread_;
    std::thread async_thread_;
    std::vector<uint8_t> buff_;

    std::vector<int> reg_return_;
    int key_;
    int version_;

    bool rtc_flag_;
    std::tm rtc_timeinfo_; 
    uint16_t rtc_milliseconds_;
    rclcpp::Publisher< builtin_interfaces::msg::Time>::SharedPtr rtc_pub_;
    builtin_interfaces::msg::Time rtc_msg_;

    bool imu_angle_flag_;
    std::vector<double> imu_angle_degree_;
    std::vector<double> imu_orientation_covariance_;

    bool imu_acceleration_flag_;
    std::vector<double> imu_acceleration_;
    std::vector<double> imu_linear_acceleration_covariance_;

    bool imu_velocity_flag_;
    std::vector<double> imu_angular_velocity_;
    std::vector<double> imu_angular_velocity_covariance_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    sensor_msgs::msg::Imu imu_msg_;

    double imu_temperature_;
    double imu_temperature_variance_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr imu_temperature_pub_;
    sensor_msgs::msg::Temperature imu_temperature_msg_;

    bool magnetometer_flag_;

    std::vector<int16_t> magnetometer_field_;
    std::vector<double> magnetometer_covariance_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetometer_pub_;
    sensor_msgs::msg::MagneticField magnetometer_msg_;

    double magnetometer_temperature_;
    double magnetometer_temperature_variance_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr magnetometer_temperature_pub_;
    sensor_msgs::msg::Temperature magnetometer_temperature_msg_;

    bool barometer_flag_;

    double barometer_pressure_;
    double barometer_variance_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr barometer_pub_;
    sensor_msgs::msg::FluidPressure barometer_msg_;

    double barometer_altitude_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr altitude_pub_;
    std_msgs::msg::Float64 altitude_msg_;


    bool gps_flag_;

    double gps_longitude_;
    double gps_latitude_;
    double gps_altitude_;    
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    sensor_msgs::msg::NavSatFix gps_msg_;

    double gps_speed_;
    double gps_heading_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr gps_ground_speed_pub_;
    geometry_msgs::msg::Twist gps_ground_speed_msg_;

    int gps_satellite_number_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr gps_satellites_pub_;
    std_msgs::msg::UInt32 gps_satellites_msg_;

    std::vector<double> gps_variance_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gps_variance_pub_;
    geometry_msgs::msg::Vector3 gps_variance_msg_;

    bool quaternion_flag_;

    std::vector<double> quaternion_;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr orientation_pub_;
    geometry_msgs::msg::Quaternion orientation_msg_;

    std::string frame_id_;
    std::string topic_name_;

    WITMotionHandler handler_;

    rclcpp::TimerBase::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

    std::vector<int> calibuff_;
    std::vector<int> mag_offset_;
    std::vector<int> mag_range_;
};

#endif // WIT_TTL_ROS2__WITMOTION_NODE_HPP_
