#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class TurtleBot3Controller(Node):
    def __init__(self):
        super().__init__('turtlebot3_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription_ = self.create_subscription(
            LaserScan,
            'scan',  # Update with the actual laser topic name
            self.laser_callback,
            10)
        self.cmd_vel_ = Twist()


    def laser_callback(self, msg):
        # Access the minimum range value from the laser scan data
        min_range = 0.3

        # Access the array of range measurements from the laser scan data
        ranges = msg.ranges

        # Find the minimum distance from the range measurements
        min_distance = min(ranges)

        if min_distance < min_range:
            # Stop the robot if an obstacle (wall) is detected
            self.cmd_vel_.linear.x = 0.0
            self.cmd_vel_.angular.z = 0.0
        else:
            # If no obstacle, continue moving forward
            self.cmd_vel_.linear.x = 0.2  # Adjust linear velocity as needed
            self.cmd_vel_.angular.z = 0.0

        # Publish the updated cmd_vel commands
        self.publisher_.publish(self.cmd_vel_)

def main(args=None):
    rclpy.init(args=args)
    turtlebot3_controller = TurtleBot3Controller()

    try:
        rclpy.spin(turtlebot3_controller)
    except KeyboardInterrupt:
        pass

    turtlebot3_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
