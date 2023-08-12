#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleBot3Controller(Node):
    def __init__(self):
        super().__init__('turtlebot3_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_ = self.create_timer(0.1, self.publish_cmd_vel)
        self.cmd_vel_ = Twist()

    def publish_cmd_vel(self):
        self.publisher_.publish(self.cmd_vel_)

    def set_linear_velocity(self, linear_x, linear_y, linear_z):
        self.cmd_vel_.linear.x = linear_x
        self.cmd_vel_.linear.y = linear_y
        self.cmd_vel_.linear.z = linear_z

    def set_angular_velocity(self, angular_x, angular_y, angular_z):
        self.cmd_vel_.angular.x = angular_x
        self.cmd_vel_.angular.y = angular_y
        self.cmd_vel_.angular.z = angular_z

def main(args=None):
    rclpy.init(args=args)
    turtlebot3_controller = TurtleBot3Controller()
    d=0
    # Set the desired linear and angular velocities here
    linear_x = 0.0  # Adjust the value to control forward/backward motion
    angular_z = 2.0  # Adjust the value to control the rotation
    linear_x_step = 0.1  # Incremental step for linear velocity
    max_linear_x = 0.2  # Maximum linear velocity
    angular_z = 2.0  # Adjust the value to control the rotation

    while linear_x <= max_linear_x:
        turtlebot3_controller.set_linear_velocity(linear_x, 0.0, 0.0)
        #turtlebot3_controller.set_angular_velocity(0.0, 0.0, angular_z)
        turtlebot3_controller.publish_cmd_vel()
        
        linear_x += linear_x_step  # Increase linear velocity in steps
        time.sleep(1)  # Wait for 1 second before the next step
    turtlebot3_controller.set_linear_velocity(0.0, 0.0, 0.0)
    if linear_x>=max_linear_x:
     turtlebot3_controller.set_angular_velocity(0.0, 0.0, angular_z)
     turtlebot3_controller.publish_cmd_vel()
      

    

    # turtlebot3_controller.set_linear_velocity(linear_x, 0.0, 0.0)
    # turtlebot3_controller.set_angular_velocity(0.0, 0.0, angular_z)

    try:
        rclpy.spin(turtlebot3_controller)
    except KeyboardInterrupt:
        turtlebot3_controller.set_linear_velocity(0.0, 0.0, 0.0)
        turtlebot3_controller.set_angular_velocity(0.0, 0.0, 0.0)
        turtlebot3_controller.publish_cmd_vel()  # Publish the stop command
        turtlebot3_controller.get_logger().info('Stopping the TurtleBot3.')


    turtlebot3_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
