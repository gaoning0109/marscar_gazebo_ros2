#!/usr/bin/env python3
import rclpy

from sensor_msgs.msg import Joy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import WrenchStamped
from gazebo_msgs.srv import SetLinkState
from std_msgs.msg import Int32,Float64MultiArray
# gazebo_msgs/srv/ApplyLinkWrench 
#     ros2 topic pub /demo/cmd_demo geometry_msgs/Twist '{linear: {x: 1.0}}' -1
vec=Vector3()

msg  = Twist()   
Wre  =  WrenchStamped()
Wre.wrench.force.x=0.0
Wre.wrench.force.y=0.0
Wre.wrench.force.z=0.0
Wre.wrench.torque.x=0.0
Wre.wrench.torque.y=0.0
Wre.wrench.torque.z=0.0

total_force  = Float64MultiArray()   
total_force.data=[0.0,0.0,0.0]
class MinimalClient(Node):

    def __init__(self):
        super().__init__('observe_py')
        self.pub_ = self.create_publisher(Twist,"/cube/cmd_demo",30)
        self.subscription = self.create_subscription(Joy,'/joy',self.callback,30)


    def callback(self,joy):
            print("k")
            if joy.buttons[9]==1:
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