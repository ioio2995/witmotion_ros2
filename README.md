# witmotion_ros2

## Description
The `witmotion_ros2` package is a ROS2 interface for WitMotion sensors, allowing you to retrieve and publish sensor data as ROS2 messages.

## Asio for Serial Port Management
This package uses the `asio` library for serial port management to handle asynchronous I/O operations. `asio` is a cross-platform C++ library for network and low-level I/O programming that provides developers with a consistent asynchronous model using modern C++ techniques.

## Installation
To install this package, follow the steps below:

1. Clone this repository into your ROS2 workspace:
   ```sh
   git clone https://github.com/ioio2995/witmotion_ros2.git
   ```
2. Install dependencies:
   ```sh
   rosdep install --from-paths src --ignore-src -r -y
   ```
3. Build the package:
   ```sh
   colcon build
   ```

## Usage
To use this package, you can either launch the node with a launch file or directly using `ros2 run` with specific arguments.

### Using Launch File
Launch the main node as follows:
```sh
ros2 launch witmotion_ros2 witmotion_launch.py
```

### Using `ros2 run`
You can also run the node directly using `ros2 run` with the following arguments:
```sh
ros2 run witmotion_ros2 witmotion_node --ros-args -p port:=/dev/ttyUSB1 -p baud_rate:=115200 -p update_rate:=50.0 -p frame_id:=base_link -p topic_name:=/witmotion
```

## Configuration
The configuration parameters are found in the `witmotion_launch.py` launch file. You can adjust them as needed. Below is an example configuration in YAML format:

```yaml
witmotion_node:
  ros__parameters:
    port: /dev/ttyUSB1
    baud_rate: 115200 # baud
    update_rate: 50.0 # Hz
    topic_name: /witmotion
    frame_id: base_link
    imu_linear_acceleration_covariance: [0.0364, 0.0, 0.0, 0.0, 0.0048, 0.0, 0.0, 0.0, 0.0796]
    imu_angular_velocity_covariance: [0.0663, 0.0, 0.0, 0.0, 0.1453, 0.0, 0.0, 0.0, 0.0378]
    imu_orientation_covariance: [0.0479, 0.0, 0.0, 0.0, 0.0207, 0.0, 0.0, 0.0, 0.0041]
    imu_temperature_variance: 0.01829
    magnetometer_covariance: [0.000000187123, 0.0, 0.0, 0.0, 0.000000105373, 0.0, 0.0, 0.0, 0.000000165816]
    magnetometer_temperature_variance: 0.01829
    barometer_variance: 0.001
```

## Nodes
### witmotion_node
This node connects to WitMotion sensors and publishes data to ROS2 topics.

#### Parameters
- `port` (string): Serial port used for connection.
- `baud_rate` (int): Baud rate of the serial connection.
- `update_rate` (double): Update rate in Hz.
- `frame_id` (string): Frame of reference for messages.
- `topic_name` (string): Topic name for publishing.

#### Published Topics
The topics are created based on the messages enabled on the sensor:
- `/witmotion_node/imu` (sensor_msgs/Imu): IMU data.
- `/witmotion_node/imu_temperature` (sensor_msgs/Temperature): IMU temperature.
- `/witmotion_node/magnetometer` (sensor_msgs/MagneticField): Magnetic field.
- `/witmotion_node/magnetometer_temperature` (sensor_msgs/Temperature): Magnetometer temperature.
- `/witmotion_node/barometer` (sensor_msgs/FluidPressure): Barometric pressure.
- `/witmotion_node/altitude` (std_msgs/Float64): Altitude.
- `/witmotion_node/gps_location` (sensor_msgs/NavSatFix): GPS location.
- `/witmotion_node/gps_ground_speed` (geometry_msgs/Twist): GPS ground speed.
- `/witmotion_node/gps_satellite_number` (std_msgs/UInt32): Number of GPS satellites.
- `/witmotion_node/gps_variance` (geometry_msgs/Vector3): GPS variance.
- `/witmotion_node/orientation` (geometry_msgs/Quaternion): Orientation.
- `/witmotion_node/rtc` (builtin_interfaces/msg/Time): Real-time clock data.

## Contributing
Contributions are welcome. Please submit a pull request for any features or bug fixes.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Authors
- [ioio2995](https://github.com/ioio2995)