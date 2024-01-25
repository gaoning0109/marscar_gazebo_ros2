#!/usr/bin/env python3
...
import rclpy
from rclpy.node import Node
from rclpy.node import Node
from copy import deepcopy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
import termios
import tty
import os
import select
import sys
import rclpy
from geometry_msgs.msg import Twist
drive  = Float64MultiArray()   
drive.data=[0.0,0.0,0.0,0.0]
vel  = Float64MultiArray()   
vel.data=[0.0,0.0,0.0,0.0,0.0,0.0]
gain = 10.0
class Control(Node):

    def __init__(self):
        super().__init__('Mars_cmdvel_Control')
        self.subscription = self.create_subscription(Twist,'/cmd_vel_mux_out',self.listener_callback,30)
        self.publisher_drive = self.create_publisher(Float64MultiArray, '/mars_car_controllers/commands',30)
        self.publisher_vel = self.create_publisher(Float64MultiArray, '/mars_car_wheel_controllers/commands',30)

    def listener_callback(self,twist):
            # print(twist.linear)
            # print(twist.angular)
            if twist.linear.x>=0:
                if twist.angular.z>=0:
                    vel.data=[gain*twist.linear.x, gain*(twist.linear.x-abs(twist.angular.z)), gain*twist.linear.x, gain*twist.linear.x, gain*(twist.linear.x+abs(twist.angular.z)), gain*twist.linear.x]
                    drive.data=[twist.angular.z,-twist.angular.z,twist.angular.z,-twist.angular.z]
                    self.publisher_drive.publish(drive)
                    self.publisher_vel.publish(vel)
                elif twist.angular.z<0:
                    vel.data=[gain*twist.linear.x, gain*(twist.linear.x+abs(twist.angular.z)), gain*twist.linear.x, gain*twist.linear.x, gain*(twist.linear.x-abs(twist.angular.z)), gain*twist.linear.x]
                    drive.data=[twist.angular.z,-twist.angular.z,twist.angular.z,-twist.angular.z]
                    self.publisher_drive.publish(drive)
                    self.publisher_vel.publish(vel)
            else:
                if twist.angular.z>=0:
                    vel.data=[gain*twist.linear.x, gain*(twist.linear.x-abs(twist.angular.z)), gain*twist.linear.x, gain*twist.linear.x, gain*(twist.linear.x+abs(twist.angular.z)), gain*twist.linear.x]
                    drive.data=[-twist.angular.z,twist.angular.z,-twist.angular.z,twist.angular.z]
                    self.publisher_drive.publish(drive)
                    self.publisher_vel.publish(vel)
                elif twist.angular.z<0:
                    vel.data=[gain*twist.linear.x, gain*(twist.linear.x+abs(twist.angular.z)), gain*twist.linear.x, gain*twist.linear.x, gain*(twist.linear.x-abs(twist.angular.z)), gain*twist.linear.x]
                    drive.data=[-twist.angular.z,twist.angular.z,-twist.angular.z,twist.angular.z]
                    self.publisher_drive.publish(drive)
                    self.publisher_vel.publish(vel)


def main(args=None):
    rclpy.init(args=args)
    control = Control()
    rclpy.spin(control)
    rclpy.shutdown()

    
    
if __name__ == '__main__':
    main()
