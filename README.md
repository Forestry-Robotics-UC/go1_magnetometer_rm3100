# RM3100 Magnetometer Driver (DroneCAN)

This ROS 2 package provides a driver for the RM3100 magnetometer using the DroneCAN protocol. It reads magnetic field data, calculates the azimuth (heading), and estimates the variance of the heading in real-time based on sensor noise.

## Features

*   **DroneCAN Interface**: Communicates with the sensor via CAN bus using DroneCAN.
*   **Coordinate Frame Conversion**: Supports both **ENU** (East-North-Up, standard ROS) and **NED** (North-East-Down) frames.
*   **Flexible Output**: Can publish heading in **Degrees** or **Radians**, referenced to **True North** (Geographic) or **Magnetic North**.
*   **Dynamic Variance Estimation**: Calculates the variance of the azimuth angle in real-time using error propagation from the raw X/Y sensor noise.
*   **Covariance Support**: Populates the ROS `MagneticField` message covariance matrix.
*   **Normalized Output**: Azimuth is normalized to the range `[-180, 180]` degrees (or `[-pi, pi]` radians).

## Dependencies

*   ROS 2 (Humble/Iron/Jazzy)
*   `dronecan` (Python package)
*   `compass_interfaces` (Custom message definitions, if applicable)
*   Standard ROS 2 message packages (`sensor_msgs`, `std_msgs`)

## Configuration

Parameters are defined in `config/params.yaml`.

### CAN Interface
*   `port`: CAN interface name (e.g., `can0`).
*   `node_id`: DroneCAN node ID for the driver.
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
*   `publish_mag`: Enable/Disable `sensor_msgs/MagneticField` publisher.
*   `publish_compass`: Enable/Disable `compass_interfaces/Azimuth` publisher.
*   `publish_azimuth`: Enable/Disable `std_msgs/Float64` azimuth publisher.
*   `publish_azimuth_var`: Enable/Disable `std_msgs/Float64` variance publisher.

## Dynamic Variance Calculation

The driver implements a sliding window buffer (size 100) to calculate the statistical variance of the raw X and Y magnetic field components in real-time.

It then uses **Error Propagation** to estimate the uncertainty ($\sigma_{\theta}^2$) of the calculated azimuth angle $\theta = \text{atan2}(y, x)$:

$$ \sigma_{\theta}^2 = \frac{y^2 \sigma_x^2 + x^2 \sigma_y^2 - 2xy \text{cov}(x,y)}{(x^2 + y^2)^2} $$

Where:
*   $\sigma_x^2, \sigma_y^2$: Variance of X and Y components.
*   $\text{cov}(x,y)$: Covariance between X and Y (currently assumed 0 in simplified implementation, but formula supports it).
*   $x, y$: Current magnetic field readings.

This allows the driver to report a higher uncertainty when the sensor is experiencing electromagnetic interference or noise.

## Usage

1.  **Build the package**:
    ```bash
    colcon build --packages-select rm3100_driver
    source install/setup.bash
    ```

2.  **Run the driver**:
    ```bash
    ros2 launch rm3100_driver rm3100_driver.launch.py
    ```

