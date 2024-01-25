#!/usr/bin/env python3
...
import rclpy
from rclpy.node import Node
from rclpy.node import Node
from copy import deepcopy
from sensor_msgs.msg import Joy

from std_msgs.msg import Float64MultiArray,Empty
from geometry_msgs.msg import Twist
cmd = Twist()

drive  = Float64MultiArray()   
drive.data=[0.0]


class Joy2cmd(Node):

    def __init__(self):
        super().__init__('joy_shooter')
        self.subscription = self.create_subscription(Joy,'/joy',self.listener_callback,10)
        self.publisher_shooter= self.create_publisher(Empty, '/my_robot/ball_shooter/fire',1)
        self.publisher_camera = self.create_publisher(Float64MultiArray, '/mars_camera_controllers/commands',30) 

    def listener_callback(self,joy):
            drive  = Float64MultiArray()   
            drive.data=[joy.axes[0]]

            self.publisher_camera.publish(drive)
            fire  = Empty()   
            if joy.axes[2]==-1 or joy.axes[5]==-1:
                self.publisher_shooter.publish(fire)

def main(args=None):
    rclpy.init(args=args)
    control = Joy2cmd()
    rclpy.spin(control)
    rclpy.shutdown()

    
    
if __name__ == '__main__':
    main()



 #/my_robot/ball_shooter/fire std_msgs/msg/Empty -1
