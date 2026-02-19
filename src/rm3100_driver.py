#!/usr/bin/env python3
"""
RM3100 Magnetometer Driver Node
This node interfaces with the RM3100 magnetometer via DroneCAN,
processes the magnetic field data, calculates azimuth (heading),
and publishes standard ROS 2 messages.
"""

import rclpy
import rclpy.node
import dronecan
from dronecan.app.node_monitor import NodeMonitor
from dronecan.app.dynamic_node_id import CentralizedServer
import select
import subprocess
import statistics
from collections import deque
from math import atan2, sin, cos
from math import pi as M_PI
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Header, Float64
from compass_interfaces.msg import Azimuth

class DroneCANDriver(rclpy.node.Node):
    """
    ROS 2 Node for handling DroneCAN magnetic field messages.
    """
    def __init__(self):
        super().__init__('rm3100_driver')
        
        # --- Parameter Declarations ---
        # CAN Interface settings
        self.port = self.declare_parameter('port', 'can0').value
        self.node_id = self.declare_parameter('node_id', 125).value
        self.bitrate = self.declare_parameter('bitrate', 1000000).value
        
        # Coordinate Frame Settings: 'ENU' (East-North-Up) or 'NED' (North-East-Down)
        self.coordinates_frame = self.declare_parameter('coordinates_frame', 'ENU').value
        
        # Reference Frame: 'GEOGRAPHIC' (True North) or 'MAGNETIC' (Magnetic North)
        self.reference = self.declare_parameter('reference', 'GEOGRAPHIC').value
        
        # Output Unit: 'DEG' (Degrees) or 'RAD' (Radians)
        self.unit = self.declare_parameter('unit', 'DEG').value
        
        # TF Frame ID
        self.frame_id = self.declare_parameter('frame_id', 'mag').value
        
        # Topic Names
        self.mag_topic = self.declare_parameter('mag_topic', 'mag').value
        self.compass_topic = self.declare_parameter('compass_topic', 'mag/compass').value
        self.azimuth_topic = self.declare_parameter('azimuth_topic', 'mag/compass/azimuth').value
        self.azimuth_var_topic = self.declare_parameter('azimuth_var_topic', 'mag/compass/azimuth_var').value
        
        # Publication Toggles
        self.publish_mag = self.declare_parameter('publish_mag', True).value
        self.publish_compass = self.declare_parameter('publish_compass', True).value
        self.publish_azimuth = self.declare_parameter('publish_azimuth', True).value
        self.publish_azimuth_var = self.declare_parameter('publish_azimuth_var', True).value
        
        # Magnetic Declination Offset (to correct Magnetic North to True North)
        self.magnetic_declination_offset = self.declare_parameter('magnetic_declination_offset', 0.5).value

        # Azimuth Offset (to account for magnetometer disalignment)
        self.azimuth_offset = self.declare_parameter('azimuth_offset', 0.0).value
        
        # Static Covariance Matrix (used if dynamic calculation is not applicable)
        self.mag_covariance = self.declare_parameter('mag_covariance', [0.0, 0.0, 0.0,
                                                                        0.0, 0.0, 0.0,
                                                                        0.0, 0.0, 0.0]).value
                
        self.mag_yml = None
        self.mag_pub = None
        self.compass_pub = None
        self.azimuth_pub = None
        self.azimuth_var_pub = None
        
        # Message Objects
        self.mag_msg = MagneticField()
        self.mag_compass = Azimuth()
        self.mag_azimuth = Float64()
        self.mag_azimuth_var = Float64()
        
        # Buffers for dynamic variance calculation (sliding window of 100 samples)
        self.mag_x_buffer = deque(maxlen=100)
        self.mag_y_buffer = deque(maxlen=100)
        
        # --- DroneCAN Initialization ---
        # Initialize the node and add a handler for MagneticFieldStrength messages
        self.node = dronecan.make_node(self.port, node_id=self.node_id, bitrate=self.bitrate)
        
        # Initialize Dynamic Node ID Allocator
        # This allows the driver to automatically assign IDs to unconfigured sensors
        self.node_monitor = NodeMonitor(self.node)
        self.allocator = CentralizedServer(self.node, self.node_monitor)
        
        self.node.add_handler(dronecan.uavcan.equipment.ahrs.MagneticFieldStrength, self.mag_handler)
        
        self.get_logger().info(f'DroneCAN Driver started with Node ID: {self.node_id}')
        self.get_logger().info('Dynamic Node ID Allocator enabled')

        # --- Publisher Initialization ---
        if self.publish_mag:
            self.mag_pub = self.create_publisher(MagneticField, self.mag_topic, 10)
        else:
            self.mag_pub = None
            
        if self.publish_compass:
            self.compass_pub = self.create_publisher(Azimuth, self.compass_topic, 10)
        else:
            self.compass_pub = None
            
        if self.publish_azimuth:
            self.azimuth_pub = self.create_publisher(Float64, self.azimuth_topic, 10)
        else:
            self.azimuth_pub = None
            
        if self.publish_azimuth_var:
            self.azimuth_var_pub = self.create_publisher(Float64, self.azimuth_var_topic, 10)
        else:
            self.azimuth_var_pub = None

    def mag_handler(self, event):
        """
        Callback function for DroneCAN magnetic field messages.
        Processes raw data, converts frames, calculates azimuth and variance.
        """
        msg = event.transfer.payload
        
        # Create Header with current timestamp
        self.h = Header()
        self.h.stamp = self.get_clock().now().to_msg()
        self.h.frame_id = self.frame_id
        
        self.mag_msg.header = self.h
        self.mag_compass.header = self.h
        
        # --- Coordinate Frame Conversion ---
        # Convert raw Gauss values to Tesla (1 Gauss = 1e-4 Tesla)
        # and adjust axes based on ENU/NED selection.
        if self.coordinates_frame == 'ENU':
            self.mag_compass.orientation = 0  # ENU 0, NED 1
            self.mag_msg.magnetic_field.x = -msg.magnetic_field_ga[0] * 1e-4 # -X
            self.mag_msg.magnetic_field.y = -msg.magnetic_field_ga[1] * 1e-4 # -Y
            self.mag_msg.magnetic_field.z = -msg.magnetic_field_ga[2] * 1e-4 # -Z
        # The Y and Z axes are negative because the sensor is upside down. The original way is to use the "MATEKSYS" brand face down, but we are using face up.
        elif self.coordinates_frame == 'NED':
            self.mag_compass.orientation = 1  # ENU 0, NED 1
            self.mag_msg.magnetic_field.x = -msg.magnetic_field_ga[1] * 1e-4 # -Y
            self.mag_msg.magnetic_field.y = msg.magnetic_field_ga[0] * 1e-4 # X
            self.mag_msg.magnetic_field.z = msg.magnetic_field_ga[2] * 1e-4 # Z

        # Apply Azimuth Offset to the magnetic field vector
        # Convert offset to radians
        theta = self.azimuth_offset * M_PI / 180.0
        
        # Rotate x and y components
        # New X = x * cos(theta) - y * sin(theta)
        # New Y = x * sin(theta) + y * cos(theta)
        
        original_mag_x = self.mag_msg.magnetic_field.x
        original_mag_y = self.mag_msg.magnetic_field.y
        
        self.mag_msg.magnetic_field.x = original_mag_x * cos(theta) - original_mag_y * sin(theta)
        self.mag_msg.magnetic_field.y = original_mag_x * sin(theta) + original_mag_y * cos(theta)
            
        self.mag_msg.magnetic_field_covariance = self.mag_covariance
        
        # --- Azimuth Calculation ---
        # Calculate heading based on Reference (Geographic/Magnetic) and Unit (Deg/Rad)
        if self.reference == 'GEOGRAPHIC':
            self.mag_compass.reference = 1 # GEOGRAPHIC 1, MAGNETIC 0
            
            if self.unit == 'DEG':
                self.mag_compass.unit = 1 # DEG 1, RAD 0
                # Calculate azimuth in degrees and add declination offset
                self.mag_compass.azimuth = (atan2(
                    self.mag_msg.magnetic_field.x,
                    self.mag_msg.magnetic_field.y) * 180.0 / M_PI) + self.magnetic_declination_offset
            
            elif self.unit == 'RAD':
                self.mag_compass.unit = 0 # DEG 1, RAD 0
                # Calculate azimuth in radians and add declination offset
                self.mag_compass.azimuth = atan2(
                    self.mag_msg.magnetic_field.x,
                    self.mag_msg.magnetic_field.y) + (self.magnetic_declination_offset * M_PI / 180.0)
        
        elif self.reference == 'MAGNETIC':  
            self.mag_compass.reference = 0 # GEOGRAPHIC 1, MAGNETIC 0
            
            if self.unit == 'DEG':
                self.mag_compass.unit = 1 # DEG 1, RAD 0
                # Calculate azimuth in degrees
                self.mag_compass.azimuth = atan2(
                    self.mag_msg.magnetic_field.x,
                    self.mag_msg.magnetic_field.y) * 180.0 / M_PI
            
            elif self.unit == 'RAD':
                self.mag_compass.unit = 0 # DEG 1, RAD 0
                # Calculate azimuth in radians
                self.mag_compass.azimuth = atan2(
                    self.mag_msg.magnetic_field.x,
                    self.mag_msg.magnetic_field.y)
        
        # Normalize azimuth to [-180, 180) degrees or [-pi, pi) radians
        if self.unit == 'DEG':
            self.mag_compass.azimuth = (self.mag_compass.azimuth + 180.0) % 360.0 - 180.0
        else:
            self.mag_compass.azimuth = (self.mag_compass.azimuth + M_PI) % (2 * M_PI) - M_PI
        
        # --- Dynamic Variance Calculation ---
        # Update buffers with latest readings
        self.mag_x_buffer.append(self.mag_msg.magnetic_field.x)
        self.mag_y_buffer.append(self.mag_msg.magnetic_field.y)
        
        # Calculate variance if we have enough samples (>5)
        if len(self.mag_x_buffer) > 5:
            # Calculate mean field (reduces noise impact)
            #mx = statistics.mean(self.mag_x_buffer)
            #my = statistics.mean(self.mag_y_buffer)
            mx = self.mag_msg.magnetic_field.x
            my = self.mag_msg.magnetic_field.y
            
            # Calculate variance of the raw sensor data stream
            var_x = statistics.variance(self.mag_x_buffer)
            var_y = statistics.variance(self.mag_y_buffer)
            
            horizontal_intensity = mx**2 + my**2
            
            if horizontal_intensity > 1e-9:
                # Error Propagation for Azimuth Variance:
                # sigma_theta^2 = ( (dy/dx * sigma_x)^2 + (dx/dy * sigma_y)^2 ) / (x^2 + y^2)^2
                # This formula estimates the variance of the angle based on the variance of x and y and their covariance
                self.mag_compass.variance = (
                    (my**2 * var_x + mx**2 * var_y)
                    / (horizontal_intensity ** 2)    
                )
            else:
                self.mag_compass.variance = 0.0
        else:
            self.mag_compass.variance = 0.0
        
        # --- Publishing ---
        if self.azimuth_pub:
            self.mag_azimuth.data = self.mag_compass.azimuth
            self.azimuth_pub.publish(self.mag_azimuth)
            
        if self.azimuth_var_pub:
            self.mag_azimuth_var.data = self.mag_compass.variance
            self.azimuth_var_pub.publish(self.mag_azimuth_var)
        
        if self.mag_pub:
            self.mag_pub.publish(self.mag_msg)
        
        if self.compass_pub:
            self.compass_pub.publish(self.mag_compass)

    def spin(self):
        """
        Main loop to process DroneCAN events and keep the node running.
        """
        try:
            while rclpy.ok():
                self.node.spin()
        # Handle interrupts gracefully
        except (select.error, OSError, KeyboardInterrupt):
            pass
        
        
def main(args=None):
    """
    Entry point for the node.
    Sets up the CAN interface and starts the ROS node.
    """
    
    # Bring up the CAN interface using ip link
    subprocess.run(['ip', 'link', 'set', 'can0', 'up', 'type', 'can', 'bitrate', '1000000'])
    
    # Create a ROS node and instantiate the class
    rclpy.init(args=args)
    driver = DroneCANDriver()
    driver.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
