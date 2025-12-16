# RM3100 Magnetometer Driver ROS2 wrapper

This ROS 2 wrapper provides a driver for the RM3100 magnetometer using the DroneCAN protocol. It reads magnetic field data, calculates the azimuth (heading), and estimates the variance of the heading in real-time. 

## Features

*   **DroneCAN Interface**: Communicates with the sensor via CAN bus using DroneCAN.
*   **Coordinate Frame Conversion**: Supports both **ENU** (East-North-Up, standard ROS) and **NED** (North-East-Down) frames.
*   **Flexible Output**: Can publish heading in **Degrees** or **Radians**, referenced to **True North** (Geographic) or **Magnetic North**.
*   **Dynamic Variance Estimation**: Calculates the variance of the azimuth angle in real-time using error propagation from the raw X/Y sensor noise.
*   **Normalized Output**: Azimuth is normalized to the range `[-180, 180]` degrees (or `[-pi, pi]` radians).

## Dependencies

*   ROS2
*   `dronecan` (Python package)

### ROS Dependencies
```bash
sudo apt install ros-$ROS_DISTRO-angles ros-$ROS_DISTRO-topic-tools ros-$ROS_DISTRO-sensor-msgs ros-$ROS_DISTRO-std-msgs
```

### System Dependencies
```bash
sudo apt install libgeographiclib-dev libcxxopts-dev 
```

### Source Dependencies
Clone these repositories into your workspace `src` folder:
```bash
git clone -b ros2 https://github.com/ctu-vras/compass.git
git clone https://github.com/TartanLlama/expected.git
git clone -b ros2 https://github.com/ctu-vras/ros-utils.git
git clone https://github.com/ctu-vras/cras_msgs.git
```

## Configuration

Parameters are defined in `config/params.yaml`.

### CAN Interface
*   `port`: CAN interface name (e.g., `can0`).
*   `node_id`: DroneCAN node ID for the driver. You do not need to change this parameter.
*   `bitrate`: CAN bus bitrate (default: 1000000).

### Frames & Reference
*   `coordinates_frame`: `'ENU'` or `'NED'`.
*   `reference`: `'GEOGRAPHIC'` (True North) or `'MAGNETIC'`.
*   `unit`: `'DEG'` or `'RAD'`.
*   `magnetic_declination_offset`: Offset to correct Magnetic North to True North (in degrees).

### Topics
*   `mag_topic`: Topic for `sensor_msgs/MagneticField` (default: `mag`).
*   `compass_topic`: Topic for `compass_interfaces/Azimuth` (default: `mag/compass`).
*   `azimuth_topic`: Topic for simple float heading (default: `mag/compass/azimuth`).
*   `azimuth_var_topic`: Topic for heading variance (default: `mag/compass/azimuth_var`).

### Publication Toggles
*   `publish_mag`: Enable/Disable `sensor_msgs/MagneticField` publisher (default: `true`).
*   `publish_compass`: Enable/Disable `compass_interfaces/Azimuth` publisher (default: `true`).
*   `publish_azimuth`: Enable/Disable `std_msgs/Float64` azimuth publisher (default: `true`).
*   `publish_azimuth_var`: Enable/Disable `std_msgs/Float64` variance publisher (default: `true`).

## Usage

1.  **Build the package**:
    *Note: Do not use `--symlink-install` as it is incompatible with some of the source dependencies.*
    ```bash
    colcon build
    source install/setup.bash
    ```

2.  **Run the driver**:
    ```bash
    ros2 launch rm3100_driver rm3100_driver.launch.py
    ```

