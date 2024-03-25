#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from copy import deepcopy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class Publisher(Node):
    
    def __init__(self):
        super().__init__('Publisher')
    
        self.publisher_ = self.create_publisher(String, '/ros2/Publisher', 30)
        self.my_timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        msg=String()
        msg.data="wawawa"
        self.publisher_.publish(msg) 

def main(args=None):
    rclpy.init(args=args)
    p = Publisher()
    rclpy.spin(p)
    rclpy.shutdown()

    
    
if __name__ == '__main__':
    main()
