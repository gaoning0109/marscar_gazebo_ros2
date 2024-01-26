#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import Joy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# gazebo_msgs/srv/ApplyLinkWrench 
#     ros2 topic pub /demo/cmd_demo geometry_msgs/Twist '{linear: {x: 1.0}}' -1

msg  = Twist()   

class MinimalClient(Node):

    def __init__(self):
        super().__init__('observe_py')
        self.pub_ = self.create_publisher(Twist,"/cube/cmd_demo",30)
        self.subscription = self.create_subscription(Joy,'/joy',self.joycallback,30)


    def joycallback(self,joy):
            print("k")
 
            msg.linear.x=joy.axes[0]*2
            msg.linear.y=-joy.axes[1]*2
            msg.linear.z=joy.axes[4]*2
            msg.angular.x=joy.axes[6]
            msg.angular.y=joy.axes[3]
            msg.angular.z=joy.axes[7]
            self.pub_.publish(msg)
        
def main():

    rclpy.init()
    minimal_client = MinimalClient()
    rclpy.spin(minimal_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()