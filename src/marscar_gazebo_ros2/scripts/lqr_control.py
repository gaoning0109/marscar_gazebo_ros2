#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy,JointState
from math import sin,cos
from distutils.log import error
import math
import numpy as np
import scipy
from control import lqr
from std_msgs.msg import Float64MultiArray
import threading
import queue
import time

def keyboard_listener():
    while True:
        try:
            user_input = input("请输入一个数字(): ")
            if user_input.lower() == 'q':
                break
            xs[0][0] = float(user_input)
            print("您输入的数字是:", xs[0][0])
        except ValueError:
            print("无效输入！请输入一个数字.")

msg  = Float64MultiArray()   
msg.data=[0.0,0.0]
m1 = 0.5
m2 = 0.5
l = 0.5
g = 9.81
A = np.array([[0.0, 0.0, 1.0, 0.0],
              [0.0, 0.0, 0.0, 1.0],
              [0.0, m2*g/m1, 0.0, 0.0],
              [0.0, (m1+m2)*g/(m1*l), 0.0, 0.0]])
B = np.array([[0.0],
              [0.0],
              [1/m1],
              [1/(m1*l)]])
Q = np.array([[100.0, 0.0, 0.0, 0.0],
              [0.0, 10.0, 0.0, 0.0],
              [0.0, 0.0, 1.0, 0.0],
              [0.0, 0.0, 0.0, 1.0]])

R = np.array([[0.01]])
K, S, E = lqr(A, B, Q, R)
print(K)
# 初始化系统状态向量 x
x = np.array([[0.0],
              [0.0],
              [0.0],  # 初始角度可能为 -π
              [2.0]])     # 初始角速度未知，这里假设为 1 rad/s

# 初始化系统状态参考值 xs
xs = np.array([[0.0],  
               [0.0],
               [0.0],
               [0.0]])
integral_limit=1000
class Control(Node):
    
    def __init__(self):
        super().__init__('Control')
        self.subscription = self.create_subscription(JointState,'/joint_states',self.listener_callback,30)
        self.publisher_ = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 30)      
        keyboard_thread = threading.Thread(target=keyboard_listener)
        keyboard_thread.start()
        self.integral_error = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def listener_callback(self,jointstate):
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time
        x[0][0]=-jointstate.position[0]
        x[1][0]=jointstate.position[1]
        x[2][0]=jointstate.velocity[0]
        x[3][0]=jointstate.velocity[1]

        # 计算误差
        error = xs[1][0] - x[1][0]  # 假设我们只关心第2位置的误差
        # 积分误差更新
        self.integral_error += error * dt

        # 积分增益Ki
        Ki = 1550  # 请在这里填入适当的积分增益

        # 计算带有积分环节的控制输入
        uo = -K@(xs - x)  # 计算比例环节的控制输入
        ui = -Ki * self.integral_error  # 计算积分环节的控制输入

        # 合成PI控制输入
        u = uo[0][0] + ui

        # 限制积分误差防止饱和
        self.integral_error = np.clip(self.integral_error, -integral_limit, integral_limit)

        msg.data[0] = u
        self.publisher_.publish(msg)

        # 更新上一时刻误差
        self.prev_error = error
def main(args=None):
    rclpy.init(args=args)
    control = Control()

    rclpy.spin(control)
    rclpy.shutdown()

    
    
if __name__ == '__main__':
    main()