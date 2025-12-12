#!/usr/bin/env python3
import rclpy
import rclpy.node
import dronecan
import select
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Header

class DroneCANDriver(rclpy.node.Node):
    def __init__(self):
        super().__init__('rm3100_driver')
        bitrate = self.declare_parameter('bitrate', 1000000).value
        node_id = self.declare_parameter('node_id', 125).value
        dna_server = self.declare_parameter('dna_server', False).value
        port = self.declare_parameter('port', 'can0').value
        self.topic = self.declare_parameter('topic', 'mag').value
        self.publish_rate = self.declare_parameter('publish_rate', 10).value
        self.publish_topic = self.declare_parameter('publish_topic', True).value
        self.frame_id = self.declare_parameter('frame_id', 'mag').value
        
        self.mag_yml = None
        self.mag_pub = None
        self.mag_msg = MagneticField()
        self.mag_msg.magnetic_field_covariance = (0.0, )*9

        # Initializing a DroneCAN node instance.
        self.node = dronecan.make_node(port, node_id=node_id, bitrate=bitrate)
        self.node.add_handler(dronecan.uavcan.equipment.ahrs.MagneticFieldStrength, self.mag_handler)

        if self.publish_topic:
            self.mag_pub = self.create_publisher(MagneticField, self.topic, int(self.publish_rate))
        else:
            self.mag_pub = None

    def mag_handler(self, event):
        msg = event.transfer.payload
        self.h = Header()
        self.h.stamp = self.get_clock().now().to_msg()
        self.h.frame_id = self.frame_id
        
        self.mag_msg.header = self.h
        self.mag_msg.magnetic_field.x = msg.magnetic_field_ga[0] * 1e-4
        self.mag_msg.magnetic_field.y = msg.magnetic_field_ga[1] * 1e-4
        self.mag_msg.magnetic_field.z = msg.magnetic_field_ga[2] * 1e-4
        
        if self.mag_pub:
            self.mag_pub.publish(self.mag_msg)

    def spin(self):
        try:
            while rclpy.ok():
                self.node.spin(0.01)
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
