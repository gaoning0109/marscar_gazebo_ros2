#!/usr/bin/env python3
...
import rclpy
from rclpy.node import Node
from rclpy.node import Node
from copy import deepcopy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
cmd = Twist()

drive  = Float64MultiArray()   
drive.data=[0.0,0.0,0.0,0.0]
vel  = Float64MultiArray()   
vel.data=[0.0,0.0,0.0,0.0,0.0,0.0]

class Joy2cmd(Node):

    def __init__(self):
        super().__init__('joy2cmd')
        self.subscription = self.create_subscription(Joy,'/joy',self.listener_callback,30)
        self.publisher_twist= self.create_publisher(Twist, '/cmd_vel_joy',30)
        self.publisher_arm = self.create_publisher(Float64MultiArray, '/mars_car_arm_controllers/commands',30) 

    def listener_callback(self,joy):
            vel  = Float64MultiArray()   
            vel.data=[joy.axes[4]*1000,joy.axes[4]*1000,0.0,0.0]
            
            cmd.linear.x=joy.axes[4]
            cmd.angular.z=joy.axes[3]
            print(joy.axes[4]*100)
            self.publisher_arm.publish(vel)
def main(args=None):
    rclpy.init(args=args)
    control = Joy2cmd()
    rclpy.spin(control)
    rclpy.shutdown()

    
    
if __name__ == '__main__':
    main()
