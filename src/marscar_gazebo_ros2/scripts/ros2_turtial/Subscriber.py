#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import Joy
from rclpy.node import Node
from std_msgs.msg import String # 假设我们发布的是字符串类型的消息

class Subscription(Node):

    def __init__(self):
        super().__init__('subscription')
        self.subscription = self.create_subscription(String,'/ros2/Publisher',self.callback,30)

    def callback(self,msg):
        print(msg.data)
 

def main():
    rclpy.init()
    s = Subscription()
    rclpy.spin(s)
    rclpy.shutdown()
if __name__ == '__main__':
    main()