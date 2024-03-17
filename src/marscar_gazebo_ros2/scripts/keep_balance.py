#!/usr/bin/env python3
...
import rclpy
import math
from rclpy.node import Node
from rclpy.node import Node
from copy import deepcopy
from sensor_msgs.msg import Joy,Imu,JointState
from std_msgs.msg import Float64MultiArray
from simple_pid import PID
import tty
import os
import select
import sys
import rclpy
from geometry_msgs.msg import Twist
from quaternions import Quaternion as Quaternion 
drive  = Float64MultiArray()   
drive.data=[0.0,0.0,0.0,0.0]
pitch=-0.116
class Control(Node):

    def __init__(self):
        super().__init__('Mars_keep_balance')
        self.subscription = self.create_subscription(Imu,'/imu',self.listener_callback,30)
        self.publisher_arm = self.create_publisher(Float64MultiArray, '/mars_car_arm_controllers/commands',30) 

    def listener_callback(self,imu):
        euler=Quaternion.get_euler(imu.orientation) 

        pid = PID(600, 10, 0.1, setpoint=0.05)
        pid.output_limits= (-1000, 1000)
        control = pid(-euler[1])
        eft  = Float64MultiArray()   
        eft.data=[control,control,0.0,0.0]
        
        self.publisher_arm.publish(eft)
        # print(-euler[1],control)

def main(args=None):
    rclpy.init(args=args)
    control = Control()
    rclpy.spin(control)
    rclpy.shutdown()

    
    
if __name__ == '__main__':
    main()
