#include "witmotion_ros2/witmotion_node.hpp"

WITMotionNode::WITMotionNode()
    : Node("witmotion_node"),
      serial_port_(io_),
      stop_async_thread_(false),
      buff_(11),
      key_(0),
      version_(0),
      rtc_flag_(false),
      imu_angle_flag_(false),
      imu_angle_degree_(3),
      imu_acceleration_flag_(false),
      imu_acceleration_(3),
      imu_velocity_flag_(false),
      imu_angular_velocity_(3),
      imu_temperature_(0),
      magnetometer_flag_(false),
      magnetometer_field_(3),
      magnetometer_temperature_(0),
      barometer_flag_(false),
      barometer_pressure_(0),
      barometer_altitude_(0),
      gps_flag_(false),
      gps_longitude_(0),
      gps_latitude_(0),
      gps_altitude_(0),
      gps_speed_(0),
      gps_heading_(0),
      gps_satellite_number_(0),
      gps_variance_(3),
      quaternion_flag_(false),
      quaternion_(4),
      handler_(this->getLogger())
{
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 9600);
    this->declare_parameter<double>("update_rate", 1.0);
    this->declare_parameter<std::string>("frame_id", "base_link");
    this->declare_parameter<std::string>("topic_name", "/" + std::string(this->get_name()));
    this->declare_parameter<std::vector<double>>("imu_orientation_covariance", std::vector<double>(9, 0.0));
    this->declare_parameter<std::vector<double>>("imu_linear_acceleration_covariance", std::vector<double>(9, 0.0));
    this->declare_parameter<std::vector<double>>("imu_angular_velocity_covariance", std::vector<double>(9, 0.0));
    this->declare_parameter<double>("imu_temperature_variance", 0.0);
    this->declare_parameter<std::vector<double>>("magnetometer_covariance", std::vector<double>(9, 0.0));
    this->declare_parameter<double>("magnetometer_temperature_variance", 0.0);
    this->declare_parameter<double>("barometer_variance", 0.0);

    auto port = this->get_parameter("port").as_string();
    auto baudrate = this->get_parameter("baud_rate").as_int();
    auto rate = this->get_parameter("update_rate").as_double();
    frame_id_ = this->get_parameter("frame_id").as_string();
    topic_name_ = this->get_parameter("topic_name").as_string();
    imu_orientation_covariance_ = this->get_parameter("imu_orientation_covariance").as_double_array();
    imu_linear_acceleration_covariance_ = this->get_parameter("imu_linear_acceleration_covariance").as_double_array();
    imu_angular_velocity_covariance_ = this->get_parameter("imu_angular_velocity_covariance").as_double_array();
    imu_temperature_variance_ = this->get_parameter("imu_temperature_variance").as_double();
    magnetometer_covariance_ = this->get_parameter("magnetometer_covariance").as_double_array();
    magnetometer_temperature_variance_ = this->get_parameter("magnetometer_temperature_variance").as_double();
    barometer_variance_ = this->get_parameter("barometer_variance").as_double();
    RCLCPP_INFO(this->getLogger(), "IMU Type: Normal Port: %s baud: %ld rate: %.2f Hz", port.c_str(), baudrate, rate);

    try
    {
        serial_port_.open(port);
        serial_port_.set_option(asio::serial_port_base::baud_rate(baudrate));
        RCLCPP_INFO(this->getLogger(), "\033[32mSerial port enabled successfully...\033[0m");
    }
    catch (const asio::system_error &e)
    {
        RCLCPP_ERROR(this->getLogger(), "Failed to open the serial port: %s", e.what());
        return;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->getLogger(), "Failed to open the serial port (standard exception): %s", e.what());
        return;
    }

    subscriber_ = this->create_subscription<std_msgs::msg::String>(topic_name_ + "/cali", 10, std::bind(&WITMotionNode::callback, this, std::placeholders::_1));
    auto period = std::chrono::milliseconds(static_cast<int>(1000 / rate));
    publisher_ = this->create_wall_timer(period, std::bind(&WITMotionNode::publisher, this));

    if (!startAsyncThread())
    {
        RCLCPP_FATAL(getLogger(), "Failed to start async thread!");
        return;
    }
}

void WITMotionNode::handleSerialData(uint8_t raw_data)
{
    buff_[key_] = raw_data;
    key_ += 1;
    auto stamp = this->now();
    if (buff_[0] != 0x55)
    {
        key_ = 0;
        return;
    }

    if (key_ < 11)
    {
        return;
    }
    else
    {
        std::vector<uint8_t> data_buff(11);
        for (int i = 0; i < 11; ++i)
        {
            data_buff[i] = buff_[i];
        }
        key_ = 0; // Reset key_ for next data

        uint8_t received_checksum = data_buff[10];
        data_buff.pop_back(); // Remove checksum byte for calculation

        if (!handler_.checkSum(data_buff, received_checksum))
        {
            return;
        }

        switch (data_buff[1])                  
            {
            case 0x50:
                handler_.handleRtcTime(rtc_timeinfo_, rtc_milliseconds_, data_buff);
                rtc_flag_ = true;
                break;
            case 0x51:
                handler_.handleAcceleration(imu_acceleration_, imu_temperature_, data_buff);
                imu_acceleration_flag_ = true;
                break;
            case 0x52:
                handler_.handleAngularVelocity(imu_angular_velocity_, data_buff);
                imu_velocity_flag_ = true;
                break;
            case 0x53:
                handler_.handleAngle(imu_angle_degree_, reinterpret_cast<int16_t &>(version_), data_buff);
                imu_angle_flag_ = true;
                break;
            case 0x54:
                handler_.handleMagnetometer(magnetometer_field_, magnetometer_temperature_, data_buff, calibuff_);
                magnetometer_flag_ = true;
                break;
            case 0x56:
                handler_.handleBarometer(barometer_pressure_, barometer_altitude_, data_buff);
                barometer_flag_ = true;
                break;
            case 0x57:
                handler_.handleGPSLocation(gps_longitude_, gps_latitude_, data_buff);
                gps_flag_ = true;
                break;
            case 0x58:
                handler_.handleGPSData(gps_altitude_, gps_heading_, gps_speed_, data_buff);
                break;
            case 0x59:
                handler_.handleQuaternion(quaternion_, data_buff);
                quaternion_flag_ = true;
                break;
            case 0x5A:
                handler_.handleGPSAccuracy(gps_satellite_number_, gps_variance_, data_buff);
                break;
            case 0x5f:
                handler_.handleReadRegister(reg_return_, data_buff);
                break;
            default:
                buff_.clear();
                break;
            }
 

        if (rtc_flag_)
        {
            time_t epoch_time = mktime(const_cast<std::tm*>(&rtc_timeinfo_));
            rtc_msg_.sec = static_cast<int32_t>(epoch_time);
            rtc_msg_.nanosec =  rtc_milliseconds_ * 1e6;
        }

        if (imu_angle_flag_ && imu_acceleration_flag_ && imu_velocity_flag_)
        {
            imu_msg_.header.stamp = stamp;
            imu_msg_.header.frame_id = frame_id_;

            std::vector<float> angle_radian(3);
            for (int i = 0; i < 3; ++i)
            {
                angle_radian[i] = imu_angle_degree_[i] * M_PI / 180;
            }
            tf2::Quaternion qua;
            qua.setRPY(angle_radian[0], angle_radian[1], angle_radian[2]);

            imu_msg_.orientation.x = qua.x();
            imu_msg_.orientation.y = qua.y();
            imu_msg_.orientation.z = qua.z();
            imu_msg_.orientation.w = qua.w();
            std::copy(imu_orientation_covariance_.begin(), imu_orientation_covariance_.end(), imu_msg_.orientation_covariance.begin());

            imu_msg_.angular_velocity.x = imu_angular_velocity_[0];
            imu_msg_.angular_velocity.y = imu_angular_velocity_[1];
            imu_msg_.angular_velocity.z = imu_angular_velocity_[2];
            std::copy(imu_angular_velocity_covariance_.begin(), imu_angular_velocity_covariance_.end(), imu_msg_.angular_velocity_covariance.begin());

            imu_msg_.linear_acceleration.x = imu_acceleration_[0];
            imu_msg_.linear_acceleration.y = imu_acceleration_[1];
            imu_msg_.linear_acceleration.z = imu_acceleration_[2];
            std::copy(imu_linear_acceleration_covariance_.begin(), imu_linear_acceleration_covariance_.end(), imu_msg_.linear_acceleration_covariance.begin());

            imu_temperature_msg_.header.stamp = stamp;
            imu_temperature_msg_.header.frame_id = frame_id_;
            imu_temperature_msg_.temperature = imu_temperature_;
            imu_temperature_msg_.variance = imu_temperature_variance_;
        }

        if (magnetometer_flag_)
        {
            magnetometer_msg_.header.stamp = stamp;
            magnetometer_msg_.header.frame_id = frame_id_;

            magnetometer_msg_.magnetic_field.x = magnetometer_field_[0];
            magnetometer_msg_.magnetic_field.y = magnetometer_field_[1];
            magnetometer_msg_.magnetic_field.z = magnetometer_field_[2];
            std::copy(magnetometer_covariance_.begin(), magnetometer_covariance_.end(), magnetometer_msg_.magnetic_field_covariance.begin());

            magnetometer_temperature_msg_.header.stamp = stamp;
            magnetometer_temperature_msg_.header.frame_id = frame_id_;
            magnetometer_temperature_msg_.temperature = magnetometer_temperature_;
            magnetometer_temperature_msg_.variance = magnetometer_temperature_variance_;
        }

        if (barometer_flag_)
        {
            barometer_msg_.header.stamp = stamp;
            barometer_msg_.header.frame_id = frame_id_;
            barometer_msg_.fluid_pressure = barometer_pressure_;
            barometer_msg_.variance = barometer_variance_;

            altitude_msg_.data = barometer_altitude_;
        }

        if (gps_flag_)
        {
            gps_msg_.header.stamp = stamp;
            gps_msg_.header.frame_id = frame_id_;

            gps_msg_.longitude = gps_longitude_;
            gps_msg_.latitude = gps_latitude_;
            gps_msg_.altitude = gps_altitude_;

            gps_satellites_msg_.data = gps_satellite_number_;
            gps_variance_msg_.x = gps_variance_[0];
            gps_variance_msg_.y = gps_variance_[1];
            gps_variance_msg_.z = gps_variance_[2];

            gps_ground_speed_msg_.linear.x = gps_speed_;
            gps_ground_speed_msg_.angular.z = gps_heading_;
        }

        if (quaternion_flag_)
        {
            orientation_msg_.w = quaternion_[0];
            orientation_msg_.x = quaternion_[1];
            orientation_msg_.y = quaternion_[2];
            orientation_msg_.z = quaternion_[3];
        }
    }
}

 void WITMotionNode::publisher()
{
    if (rtc_flag_)
    {
        if (!rtc_pub_)
            rtc_pub_ = this->create_publisher< builtin_interfaces::msg::Time>(topic_name_ + "/rtc", 10);
        rtc_pub_->publish(rtc_msg_);
    }

    if (imu_angle_flag_ && imu_acceleration_flag_ && imu_velocity_flag_)
    {
        if (!imu_pub_)
            imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(topic_name_ + "/imu", 10);
        imu_pub_->publish(imu_msg_);
    }

    if (imu_acceleration_flag_)
    {
        if (!imu_temperature_pub_)
            imu_temperature_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>(topic_name_ + "/imu_temperature", 10);
        imu_temperature_pub_->publish(imu_temperature_msg_);
    }

    if (magnetometer_flag_)
    {
        if (!magnetometer_pub_)
            magnetometer_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>(topic_name_ + "/magnetometer", 10);
        magnetometer_pub_->publish(magnetometer_msg_);

        if (!magnetometer_temperature_pub_)
            magnetometer_temperature_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>(topic_name_ + "/magnetometer_temperature", 10);
        magnetometer_temperature_pub_->publish(magnetometer_temperature_msg_);
    }

    if (barometer_flag_)
    {
        if (!barometer_pub_)
            barometer_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>(topic_name_ + "/barometer", 10);
        barometer_pub_->publish(barometer_msg_);

        if (!altitude_pub_)
            altitude_pub_ = this->create_publisher<std_msgs::msg::Float64>(topic_name_ + "/altitude", 10);
        altitude_pub_->publish(altitude_msg_);        
    }

    if (gps_flag_)
    {
        if (!gps_pub_)
            gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>( topic_name_ + "/gps_location", 10);
        gps_pub_->publish(gps_msg_);

        if (!gps_ground_speed_pub_)
            gps_ground_speed_pub_ = this->create_publisher<geometry_msgs::msg::Twist>( topic_name_ + "/gps_ground_speed", 10);
        gps_ground_speed_pub_->publish(gps_ground_speed_msg_);

        if (!gps_satellites_pub_)
            gps_satellites_pub_ = this->create_publisher<std_msgs::msg::UInt32>( topic_name_ + "/gps_satellite_number", 10);
        gps_satellites_pub_->publish(gps_satellites_msg_);

        if (!gps_variance_pub_)
            gps_variance_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>( topic_name_ + "/gps_variance", 10);
        gps_variance_pub_->publish(gps_variance_msg_);
    }

    if (quaternion_flag_)
    {
        if (!orientation_pub_)
            orientation_pub_ = this->create_publisher<geometry_msgs::msg::Quaternion>( topic_name_ + "/orientation", 10);
        orientation_pub_->publish(orientation_msg_);
    }
}

void WITMotionNode::callback(const std_msgs::msg::String::SharedPtr data)
{
    std::string data_str = data->data;
    RCLCPP_INFO(this->getLogger(), "Callback: %s", data_str.c_str());

    if (data_str.find("mag") != std::string::npos)
    {   
        handler_.sendUnlockIMUCommand(serial_port_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        handler_.sendResetMagOffsetCommands(serial_port_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        handler_.sendEnterMagCalibrationCommand(serial_port_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        calibuff_.clear();
        mag_offset_ = {0, 0, 0};
        mag_range_ = {500, 500, 500};
    }
    else if (data_str.find("exti") != std::string::npos)
    {
        handler_.sendUnlockIMUCommand(serial_port_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        handler_.sendExitCalibrationCommand(serial_port_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        handler_.sendSaveParametersCommand(serial_port_);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        handler_.sendReadMagOffsetCommand(serial_port_);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::copy(reg_return_.begin(), reg_return_.end(), mag_offset_.begin());
        handler_.sendReadMagRangeCommand(serial_port_);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::copy(reg_return_.begin(), reg_return_.end(), mag_range_.begin());
        int datalen = calibuff_.size();
        RCLCPP_INFO(this->getLogger(), "cali data %d", datalen);
        std::vector<float> r;
        if (datalen > 0)
        {
            for (int i = 0; i < datalen; ++i)
            {
                float tempx = ((calibuff_[i] - mag_offset_[0]) * 2.0 / mag_range_[0]);
                float tempy = ((calibuff_[i + 1] - mag_offset_[1]) * 2.0 / mag_range_[1]);
                float temp = tempx * tempx + tempy * tempy - 1;
                r.push_back(fabs(temp));
            }
            float sumval = std::accumulate(r.begin(), r.end(), 0.0f);
            float r_n = sumval / datalen;
            if (r_n < 0.05)
            {
                RCLCPP_INFO(this->getLogger(), "magnetic field calibration results are very good");
            }
            else if (r_n < 0.1)
            {
                RCLCPP_INFO(this->getLogger(), "magnetic field calibration results are good");
            }
            else
            {
                RCLCPP_WARN(this->getLogger(), "magnetic field calibration results are bad, please try again");
            }
        }
    }
    else if (data_str.find("version") != std::string::npos)
    {   
        RCLCPP_INFO(this->getLogger(), "sensor version is %d", version_);
    }
    else if (data_str.find("rate") != std::string::npos)
    {
        try
        {
            float rate = std::stof(data_str.substr(4));
            handler_.sendSetRateCommand(serial_port_, rate);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->getLogger(), "Error changing rate: %s", e.what());
        }
    }
    else if (data_str.find("baud") != std::string::npos)
    {
        try
        {
            int baud = std::stoi(data_str.substr(4));
            handler_.sendSetBaudCommand(serial_port_, baud);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->getLogger(), "Error changing baud rate: %s", e.what());
        }
    }
    else if (data_str.find("rsw") != std::string::npos)
    {
        handler_.sendUnlockIMUCommand(serial_port_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        handler_.sendEnterMagCalibrationCommand(serial_port_);
    }
}

rclcpp::Logger WITMotionNode::getLogger()
{
    return this->get_logger();
}

void WITMotionNode::asyncThread()
{
    RCLCPP_INFO(getLogger(), "Start command thread:");
    asio::streambuf buffer;

    while (!stop_async_thread_)
    {
        if (!serial_port_.is_open())
        {
            RCLCPP_ERROR(this->getLogger(), "Serial port is not open");
            return;
        }

        try
        {
            size_t n = asio::read(serial_port_, buffer, asio::transfer_at_least(1));
            std::istream is(&buffer);

            while (is && n > 0)
            {
                uint8_t raw_data;
                is.read(reinterpret_cast<char *>(&raw_data), 1);
                handleSerialData(raw_data);
                n--;
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->getLogger(), "Exception during read: %s", e.what());
            return;
        }
    }
    return;
}

bool WITMotionNode::startAsyncThread()
{
    if (!async_thread_.joinable())
    {
        async_thread_ = std::thread(&WITMotionNode::asyncThread, this);
    }
    else
    {
        RCLCPP_WARN(getLogger(), "Could not start command thread, command thread already running!");
        return false;
    }

#ifdef WITMOTION_NODE__THREAD_PRIORITY
    std::ifstream realtime_file{"/sys/kernel/realtime", std::ios::in};
    bool has_realtime{false};
    if (realtime_file.is_open())
    {
        realtime_file >> has_realtime;
    }

    int policy{};
    struct ::sched_param param
    {
    };
    ::pthread_getschedparam(async_thread_.native_handle(), &policy, &param);
    if (has_realtime)
    {
        policy = SCHED_FIFO;
        RCLCPP_INFO(getLogger(), "Real-time system detected: Setting policy to 'SCHED_FIFO'...");
    }
    int const max_thread_priority{::sched_get_priority_max(policy)};
    if (max_thread_priority != -1)
    {
        param.sched_priority = max_thread_priority;
        if (::pthread_setschedparam(async_thread_.native_handle(), policy, &param) == 0)
        {
            RCLCPP_INFO(getLogger(), "Set thread priority '%d' and policy '%d' to async thread!",
                        param.sched_priority, policy);
        }
        else
        {
            RCLCPP_WARN(getLogger(), "Failed to set thread priority '%d' and policy '%d' to async thread!",
                        param.sched_priority, policy);
        }
    }
    else
    {
        RCLCPP_WARN(getLogger(), "Could not set thread priority to async thread: Failed to get max priority!");
    }
#endif // WITMOTION_NODE__THREAD_PRIORITY

    return true;
}

void WITMotionNode::stopAsyncThread()
{
    RCLCPP_INFO(getLogger(), "stop command thread:");
    if (async_thread_.joinable())
    {
        stop_async_thread_.store(true);
        async_thread_.join();
    }
    else
    {
        RCLCPP_WARN(getLogger(), "Could not stop command thread: Not running!");
    }
    return;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WITMotionNode>();
    rclcpp::spin(node);
    node->stopAsyncThread();
    rclcpp::shutdown();
    return 0;
}
