#ifndef WITMOTION_HANDLER_HPP
#define WITMOTION_HANDLER_HPP

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <asio.hpp>

class WITMotionHandler
{
public:
    WITMotionHandler(rclcpp::Logger logger);

    std::vector<short> hexToShortVector(const std::vector<uint8_t>& raw_data);
    int hexToInt(const std::vector<uint8_t>& raw_data);
    short hexToShort(const std::vector<uint8_t>& raw_data);
    void handleRtcTime(std::tm& timeinfo, uint16_t& milliseconds, const std::vector<uint8_t>& data_buff);
    void handleAcceleration(std::vector<double> &acceleration, double &acceleration_temperature, const std::vector<uint8_t> &data_buff);
    void handleAngularVelocity(std::vector<double>& angularVelocity, const std::vector<uint8_t>& data_buff);
    void handleAngle(std::vector<double>& angle_degree, int16_t& version, const std::vector<uint8_t>& data_buff);
    void handleMagnetometer(std::vector<int16_t> &magnetometer, double &magnetometer_temperature, const std::vector<uint8_t> &data_buff, std::vector<int>& calibuff_);    
    void handleBarometer(double &pressure, double &altitude_bar, const std::vector<uint8_t> &data_buff);
    void handleQuaternion(std::vector<double> &quaternion, const std::vector<uint8_t> &data_buff);
    void handleGPSLocation(double& longitude_gps, double& latitude_gps, const std::vector<uint8_t>& data_buff);
    void handleGPSData(double& altitude_gps, double& heading_gps, double& speed_gps, const std::vector<uint8_t>& data_buff);
    void handleGPSAccuracy(int &satellite_number, std::vector<double> &gps_variance, const std::vector<uint8_t> &data_buff);
    void handleReadRegister(std::vector<int>& reg_return, const std::vector<uint8_t>& data_buff);

    bool checkSum(const std::vector<uint8_t>& data, uint8_t check_data);
    uint8_t calculateChecksum(const std::vector<uint8_t>& data);

    void sendUnlockIMUCommand(asio::serial_port& serial_port);
    void sendResetMagOffsetCommands(asio::serial_port& serial_port);
    void sendEnterMagCalibrationCommand(asio::serial_port& serial_port);
    void sendExitCalibrationCommand(asio::serial_port& serial_port);
    void sendSaveParametersCommand(asio::serial_port& serial_port);
    void sendReadMagOffsetCommand(asio::serial_port& serial_port);
    void sendReadMagRangeCommand(asio::serial_port& serial_port);
    void sendSetRateCommand(asio::serial_port& serial_port, float rate);
    void sendSetBaudCommand(asio::serial_port& serial_port, int baud);

private:
    rclcpp::Logger logger_;
};

#endif // WITMOTION_HANDLER_HPP
