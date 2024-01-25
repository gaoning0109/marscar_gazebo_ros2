#!/usr/bin/env python3
...
import sys
import rclpy
import time
from rclpy.node import Node
from gazebo_msgs.srv import ApplyLinkWrench
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.srv import GetEntityState
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.srv import SetLinkState
from std_msgs.msg import Int32,Float64MultiArray
from marscar_gazebo_ros2.msg import Cableposseqq 
import socket
import json
import threading
# gazebo_msgs/srv/ApplyLinkWrench 
# 3.定义节点类；gazebo_msgs/srv/SetEntityState
client_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
class MinimalClient(Node):

    def __init__(self):
        super().__init__('minimal_client_py')

        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        self.cli_get = self.create_client(GetEntityState, '/get_entity_state')
        # self.subscription = self.create_subscription(Float64MultiArray,'/xyz',self.send_request,30)
        self.subscription = self.create_subscription(Cableposseqq,'/cable/cable_pub_testarray',self.send_request,30)
        # thread_serial = threading.Thread(target=MinimalClient.send_node)
        # thread_serial.start()
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务连接中，请稍候...')
        self.req = SetEntityState.Request()
        self.req_res = SetEntityState.Response()
        self.req_get = GetEntityState.Request()
        self.set_link = SetLinkState.Request()

    def send_request(self,xyz):

            for i in range(21):
                self.req.state.name="keypoints"+str(i)
                self.req.state.pose.position.x=xyz.x[i]
                self.req.state.pose.position.y=xyz.y[i]
                self.req.state.pose.position.z=xyz.z[i]
                self.req.state.reference_frame="world"
                self.future = self.cli.call_async(self.req)
                print(xyz.z[i])
                    
                time.sleep(0.001)

def main():

    rclpy.init()
    minimal_client = MinimalClient()
    rclpy.spin(minimal_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()