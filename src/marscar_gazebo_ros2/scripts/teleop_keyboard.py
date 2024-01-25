#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.node import Node
from copy import deepcopy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
msg  = Float64MultiArray()   
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


tur = """
控制你的火星车
---------------------------
控制按键如下:
        w
   a    s    d
        x

w/x : 前进后退
a/d : 左拐右拐

空格键或s : 停止

CTRL-C 退出
"""

e = """
Communications Failed
"""


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
settings = None
settings = termios.tcgetattr(sys.stdin)

msg.data=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
class Control(Node):

    def __init__(self):
        super().__init__('Mars_keyboard_Control')
        # self.subscription = self.create_subscription(Twist,'/mars_cmd_vel',self.listener_callback,30)
        self.publisher_drive = self.create_publisher(Float64MultiArray, '/mars_car_controllers/commands',30)
        self.publisher_vel = self.create_publisher(Float64MultiArray, '/mars_car_wheel_controllers/commands',30)
        self.publisher_arm = self.create_publisher(Float64MultiArray, '/mars_car_arm_controllers/commands',30) 
        try:
            print(tur)
            while(1):
                key = get_key(settings)
                if key == 'w':
                    print("前进前进！！！")
                    vel.data=[-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]
 
                elif key == 'x':
                    print("后退！")
                    vel.data=[1.0,1.0,1.0,1.0,1.0,1.0]
                elif key == 'a':
                    print("左拐！")
                    drive.data[0]=1.0
                    drive.data[1]=-1.0
                    drive.data[2]= 1.0
                    drive.data[3]= -1.0
                elif key == 'd':
                    print("右拐！")
                    drive.data[0]=-1.0
                    drive.data[1]=1.0
                    drive.data[2]= -1.0
                    drive.data[3]= 1.0
                elif key == ' ' or key == 's':
                    print("停！")
                    drive.data=[0.0,0.0,0.0,0.0]
                else:
                    if (key == '\x03'):
                        break

                self.publisher_drive.publish(drive)
                self.publisher_vel.publish(vel)
        except Exception as e:
            print(e)

def main(args=None):
    rclpy.init(args=args)
    control = Control()
    rclpy.spin(control)
    rclpy.shutdown()

    
    
if __name__ == '__main__':
    main()
