#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.node import Node
from copy import deepcopy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
msg  = Float64MultiArray()   
msg.data=[0.0,0.0]
class Control(Node):
    
    def __init__(self):
        super().__init__('Control')
        self.subscription = self.create_subscription(Joy,'/joy',self.listener_callback,30)
        self.publisher_ = self.create_publisher(Float64MultiArray, '/cart_controllers/commands', 30)
        
    def listener_callback(self,joy):
            msg.data[0]=joy.axes[0]*500
            msg.data[1]=joy.axes[1]*500
            self.publisher_.publish(msg)  
            # print("controling...",msg.data) 
def main(args=None):
    rclpy.init(args=args)
    control = Control()
    rclpy.spin(control)
    rclpy.shutdown()

    
    
if __name__ == '__main__':
    main()
