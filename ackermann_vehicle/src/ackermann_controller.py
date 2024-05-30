#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class AckermannController(Node):

    def __init__(self):
        super().__init__('ackermann_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_ = self.create_timer(1.0, self.publish_message)
        
    def publish_message(self):
        ackermann_msg = Twist()
        ackermann_msg.linear.x = 0.0
        ackermann_msg.angular.z = 0.
        self.publisher_.publish(ackermann_msg)

def main(args=None):
    rclpy.init(args=args)
    ackermann_controller = AckermannController()
    rclpy.spin(ackermann_controller)
    ackermann_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
