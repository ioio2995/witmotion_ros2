#include "witmotion_ros2/witmotion_handler.hpp"
#include "witmotion_ros2/witmotion_commands.hpp"
#include <numeric>
#include <cstring>
#include <thread>

WITMotionHandler::WITMotionHandler(rclcpp::Logger logger)
    : logger_(logger)
{
}

std::vector<short> WITMotionHandler::hexToShortVector(const std::vector<uint8_t>& raw_data)
{
    std::vector<short> result(4);
    std::memcpy(result.data(), raw_data.data(), 8);
    return result;
}

int WITMotionHandler::hexToInt(const std::vector<uint8_t>& raw_data)
{
    int result;
    std::memcpy(&result, raw_data.data(), 4);
    return result;
}

short WITMotionHandler::hexToShort(const std::vector<uint8_t>& raw_data)
{
    short result;
    std::memcpy(&result, raw_data.data(), 2);
    return result;
}

bool WITMotionHandler::checkSum(const std::vector<uint8_t>& data, uint8_t check_data)
{
    int sum = std::accumulate(data.begin(), data.end(), 0);
    bool result = (sum & 0xff) == check_data;
    if (!result)
    {
        RCLCPP_DEBUG(logger_, "Checksum failure: calculated sum = 0x%02X, expected sum = 0x%02X", (sum & 0xff), check_data);
    }
    return result;
}

uint8_t WITMotionHandler::calculateChecksum(const std::vector<uint8_t>& data)
{
    return std::accumulate(data.begin(), data.end(), 0) & 0xFF;
}

void WITMotionHandler::handleRtcTime(std::tm& timeinfo, uint16_t& milliseconds, const std::vector<uint8_t>& data_buff)
{
        timeinfo.tm_year = data_buff[2] + 100;  
        timeinfo.tm_mon = data_buff[3] - 1;     
        timeinfo.tm_mday = data_buff[4];        
        timeinfo.tm_hour = data_buff[5];        
        timeinfo.tm_min = data_buff[6];         
        timeinfo.tm_sec = data_buff[7];         
        milliseconds = hexToShort({data_buff[8], data_buff[9]});
}

void WITMotionHandler::handleAcceleration(std::vector<double> &acceleration, double &acceleration_temperature, const std::vector<uint8_t> &data_buff)
{
    std::vector<short> temp = hexToShortVector(std::vector<uint8_t>(data_buff.begin() + 2, data_buff.begin() + 10));
    for (int i = 0; i < 3; ++i)
    {
        acceleration[i] = temp[i] / 32768.0 * 16 * 9.8;
    }
    acceleration_temperature = temp[3] / 100.0;
}

void WITMotionHandler::handleAngularVelocity(std::vector<double>& angularVelocity, const std::vector<uint8_t>& data_buff)
{
    std::vector<short> temp = hexToShortVector(std::vector<uint8_t>(data_buff.begin() + 2, data_buff.begin() + 10));
    for (int i = 0; i < 3; ++i)
    {
        angularVelocity[i] = temp[i] / 32768.0 * 2000 * M_PI / 180;
    }
}

void WITMotionHandler::handleAngle(std::vector<double>& angle_degree, int16_t& version, const std::vector<uint8_t>& data_buff)
{
    std::vector<short> temp = hexToShortVector(std::vector<uint8_t>(data_buff.begin() + 2, data_buff.begin() + 10));
    for (int i = 0; i < 3; ++i)
    {
        angle_degree[i] = temp[i] / 32768.0 * 180;
    }
    version = temp[3];
}

void WITMotionHandler::handleMagnetometer(std::vector<int16_t> &magnetometer, double &magnetometer_temperature, const std::vector<uint8_t> &data_buff, std::vector<int>& calibuff)
{
    std::vector<short> temp = hexToShortVector(std::vector<uint8_t>(data_buff.begin() + 2, data_buff.begin() + 10));
    for (int i = 0; i < 3; ++i)
    {
        magnetometer[i] = temp[i];
    }
    magnetometer_temperature = temp[3] / 100.0;
    calibuff.insert(calibuff.end(), {static_cast<int>(magnetometer[0]), static_cast<int>(magnetometer[1])});
}

void WITMotionHandler::handleBarometer(double &pressure, double &bar_altitude, const std::vector<uint8_t> &data_buff)
{
    int32_t pressure_raw = hexToInt(std::vector<uint8_t>(data_buff.begin() + 2, data_buff.begin() + 6));
    int32_t altitude_raw = hexToInt(std::vector<uint8_t>(data_buff.begin() + 6, data_buff.begin() + 10));
   
    pressure = static_cast<double>(pressure_raw) / 100.0;
    bar_altitude = static_cast<double>(altitude_raw) / 100.0;
}

void WITMotionHandler::handleQuaternion(std::vector<double> &quaternion, const std::vector<uint8_t> &data_buff)
{
    std::vector<short> quaternion_raw = hexToShortVector(std::vector<uint8_t>(data_buff.begin() + 2, data_buff.begin() + 10));
    
    for (int i = 0; i < 4; ++i)
    {
        quaternion[i] = static_cast<double>(quaternion_raw[i]) / 32768.0; 
    }
}

void WITMotionHandler::handleGPSLocation(double& longitude_gps, double& latitude_gps, const std::vector<uint8_t>& data_buff)
{
    int32_t lon_raw = hexToInt(std::vector<uint8_t>(data_buff.begin() + 2, data_buff.begin() + 6));
    int32_t lat_raw = hexToInt(std::vector<uint8_t>(data_buff.begin() + 6, data_buff.begin() + 10));

    float  longitude_dms = lon_raw / 10000000.0;
    float  latitude_dms = lat_raw / 10000000.0;

    int lon_deg = static_cast<int>(longitude_dms);
    float  lon_min_full = (longitude_dms - lon_deg) * 100.0;
    int lon_min = static_cast<int>(lon_min_full);
    float  lon_sec = (lon_min_full - lon_min) * 60.0;
    longitude_gps = lon_deg + (lon_min / 60.0) + (lon_sec / 3600.0);

    int lat_deg = static_cast<int>(latitude_dms);
    float  lat_min_full = (latitude_dms - lat_deg) * 100.0;
    int lat_min = static_cast<int>(lat_min_full);
    float  lat_sec = (lat_min_full - lat_min) * 60.0;
    latitude_gps = lat_deg + (lat_min / 60.0) + (lat_sec / 3600.0);
}

void WITMotionHandler::handleGPSData(double& altitude_gps, double& heading_gps, double& speed_gps, const std::vector<uint8_t>& data_buff)
{
    altitude_gps = hexToShort(std::vector<uint8_t>(data_buff.begin() + 2, data_buff.begin() + 4)) / 10.0;
    heading_gps = hexToShort(std::vector<uint8_t>(data_buff.begin() + 4, data_buff.begin() + 6)) / 100.0;
    speed_gps = hexToInt(std::vector<uint8_t>(data_buff.begin() + 6, data_buff.begin() + 10));
}

void WITMotionHandler::handleGPSAccuracy(int &satellite_number, std::vector<double> &gps_variance, const std::vector<uint8_t> &data_buff)
{
    satellite_number = hexToShort({data_buff[2], data_buff[3]});
    
    double pdop = static_cast<double>(hexToShort({data_buff[4], data_buff[5]})) / 100.0;
    double hdop = static_cast<double>(hexToShort({data_buff[6], data_buff[7]})) / 100.0;
    double vdop = static_cast<double>(hexToShort({data_buff[8], data_buff[9]})) / 100.0;
    
    gps_variance[0] = hdop * hdop;
    gps_variance[1] = vdop * vdop;
    gps_variance[2] = pdop * pdop;
}

void WITMotionHandler::handleReadRegister(std::vector<int>& reg_return, const std::vector<uint8_t>& data_buff)
{
    std::vector<short> temp = hexToShortVector(std::vector<uint8_t>(data_buff.begin() + 2, data_buff.begin() + 10));
    reg_return = std::vector<int>(temp.begin(), temp.end());
}

void WITMotionHandler::sendUnlockIMUCommand(asio::serial_port& serial_port)
{
    serial_port.write_some(asio::buffer(WitMotionCommands::UNLOCK_IMU_CMD));
}

void WITMotionHandler::sendResetMagOffsetCommands(asio::serial_port& serial_port)
{
    serial_port.write_some(asio::buffer(WitMotionCommands::RESET_MAGX_OFFSET_CMD));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    serial_port.write_some(asio::buffer(WitMotionCommands::RESET_MAGY_OFFSET_CMD));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    serial_port.write_some(asio::buffer(WitMotionCommands::RESET_MAGZ_OFFSET_CMD));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    serial_port.write_some(asio::buffer(WitMotionCommands::RESET_MAG_PARAM_CMD));
}

void WITMotionHandler::sendEnterMagCalibrationCommand(asio::serial_port& serial_port)
{
    serial_port.write_some(asio::buffer(WitMotionCommands::ENTER_MAG_CALI_CMD));
}

void WITMotionHandler::sendExitCalibrationCommand(asio::serial_port& serial_port)
{
    serial_port.write_some(asio::buffer(WitMotionCommands::EXIT_CALI_CMD));
}

void WITMotionHandler::sendSaveParametersCommand(asio::serial_port& serial_port)
{
    serial_port.write_some(asio::buffer(WitMotionCommands::SAVE_PARAM_CMD));
}

void WITMotionHandler::sendReadMagOffsetCommand(asio::serial_port& serial_port)
{
    serial_port.write_some(asio::buffer(WitMotionCommands::READ_MAG_OFFSET_CMD));
}

void WITMotionHandler::sendReadMagRangeCommand(asio::serial_port& serial_port)
{
    serial_port.write_some(asio::buffer(WitMotionCommands::READ_MAG_RANGE_CMD));
}

void WITMotionHandler::sendSetRateCommand(asio::serial_port& serial_port, float rate)
{
    static std::vector<float> ratelist = {0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 125, 200};
    uint8_t val = 0;
    for (size_t i = 0; i < ratelist.size(); ++i)
    {
        if (rate == ratelist[i])
        {
            val = i + 1;
            break;
        }
    }
    if (val != 0)
    {
        std::vector<uint8_t> cmd = {0xff, 0xaa, 0x03, val, 0x00};
        sendUnlockIMUCommand(serial_port);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        serial_port.write_some(asio::buffer(cmd));
    }
}

void WITMotionHandler::sendSetBaudCommand(asio::serial_port& serial_port, int baud)
{
    static std::vector<int> baudlist = {4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800};
    uint8_t val = 0;
    for (size_t i = 0; i < baudlist.size(); ++i)
    {
        if (baud == baudlist[i])
        {
            val = i + 1;
            break;
        }
    }
    if (val != 0)
    {
        std::vector<uint8_t> cmd = {0xff, 0xaa, 0x04, val, 0x00};
        sendUnlockIMUCommand(serial_port);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        serial_port.write_some(asio::buffer(cmd));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        serial_port.set_option(asio::serial_port_base::baud_rate(baud));
    }
}
