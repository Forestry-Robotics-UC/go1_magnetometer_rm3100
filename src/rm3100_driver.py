#!/usr/bin/env python3
import rclpy
import rclpy.node
import dronecan
import select
from math import sqrt, atan2
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Header, Float64
from compass_interfaces.msg import Azimuth

class DroneCANDriver(rclpy.node.Node):
    def __init__(self):
        super().__init__('rm3100_driver')
        port = self.declare_parameter('port', 'can0').value
        node_id = self.declare_parameter('node_id', 125).value
        bitrate = self.declare_parameter('bitrate', 1000000).value
        self.frame_id = self.declare_parameter('frame_id', 'mag').value
        self.mag_topic = self.declare_parameter('mag_topic', 'mag').value
        self.compass_topic = self.declare_parameter('compass_topic', 'mag/compass').value
        self.azimuth_topic = self.declare_parameter('azimuth_topic', 'mag/compass/azimuth').value
        self.azimuth_var_topic = self.declare_parameter('azimuth_var_topic', 'mag/compass/azimuth_var').value
        self.publish_mag = self.declare_parameter('publish_mag', True).value
        self.publish_compass = self.declare_parameter('publish_compass', True).value
        self.publish_azimuth = self.declare_parameter('publish_azimuth', True).value
        self.publish_azimuth_var = self.declare_parameter('publish_azimuth_var', True).value
        self.magnetic_declination_offset = self.declare_parameter('magnetic_declination_offset', 16).value
        
        self.mag_yml = None
        self.mag_pub = None
        self.compass_pub = None
        self.azimuth_pub = None
        self.azimuth_var_pub = None
        self.mag_msg = MagneticField()
        self.mag_compass = Azimuth()
        self.mag_azimuth = Float64()
        self.mag_azimuth_var = Float64()
        
        # Initializing a DroneCAN node instance.
        self.node = dronecan.make_node(port, node_id=node_id, bitrate=bitrate)
        self.node.add_handler(dronecan.uavcan.equipment.ahrs.MagneticFieldStrength, self.mag_handler)

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
        msg = event.transfer.payload
        self.h = Header()
        self.h.stamp = self.get_clock().now().to_msg()
        self.h.frame_id = self.frame_id
        
        self.mag_msg.header = self.h
        self.mag_msg.magnetic_field.x = msg.magnetic_field_ga[0] * 1e-4
        self.mag_msg.magnetic_field.y = msg.magnetic_field_ga[1] * 1e-4
        self.mag_msg.magnetic_field.z = msg.magnetic_field_ga[2] * 1e-4
        self.mag_msg.magnetic_field_covariance = msg.magnetic_field_covariance
        
        self.mag_compass.header = self.h
        self.mag_compass.variance = sqrt(
            self.mag_msg.magnetic_field.x ** 2 +
            self.mag_msg.magnetic_field.y ** 2 +
            self.mag_msg.magnetic_field.z ** 2)
        
        if self.mag_msg.magnetic_field.y > 0:
            self.mag_compass.azimuth = 90 - (atan2(
                self.mag_msg.magnetic_field.x,
                self.mag_msg.magnetic_field.y) * 180.0 / 3.14159265) + self.magnetic_declination_offset
        elif self.mag_msg.magnetic_field.y < 0:
            self.mag_compass.azimuth = 270 - (atan2(
                self.mag_msg.magnetic_field.x,
                self.mag_msg.magnetic_field.y) * 180.0 / 3.14159265) + self.magnetic_declination_offset
        elif self.mag_msg.magnetic_field.y == 0 and self.mag_msg.magnetic_field.x < 0:
            self.mag_compass.azimuth = 180.0 + self.magnetic_declination_offset
        elif self.mag_msg.magnetic_field.y == 0 and self.mag_msg.magnetic_field.x > 0:
            self.mag_compass.azimuth = 0.0 + self.magnetic_declination_offset
            
        self.mag_compass.unit = 1 # DEG 1, RAD 0
        self.mag_compass.orientation = 0 # ENU 0, NED 1
        self.mag_compass.reference = 1 # GEOGRAPHIC 1, MAGNETIC 0, UTM 2
        
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
        try:
            while rclpy.ok():
                self.node.spin()
        # Ctrl-C signal interferes with select with the ROS signal handler
        # should be OSError in python 3.?
        except (select.error, OSError, KeyboardInterrupt):
            pass
        
        
def main(args=None):
    """Create a ROS node and instantiate the class."""
    rclpy.init(args=args)
    driver = DroneCANDriver()
    driver.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
