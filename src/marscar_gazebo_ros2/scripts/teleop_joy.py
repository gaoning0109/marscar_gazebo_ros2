# 发布话题 /rexrov/joy:sensor_msgs/Joy
# 根据命令控制关节运动：
import rclpy
from rclpy.node import Node
from rclpy.node import Node
from copy import deepcopy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

drive  = Float64MultiArray()   
drive.data=[0.0,0.0,0.0,0.0]
vel  = Float64MultiArray()   
vel.data=[0.0,0.0,0.0,0.0,0.0,0.0]
class Control(Node):

    def __init__(self):
        super().__init__('Control')
        self.subscription = self.create_subscription(Joy,'/joy',self.listener_callback,30)
        self.publisher_drive = self.create_publisher(Float64MultiArray, '/mars_car_controllers/commands',30)
        self.publisher_vel = self.create_publisher(Float64MultiArray, '/mars_car_wheel_controllers/commands',30)
        self.publisher_arm = self.create_publisher(Float64MultiArray, '/mars_car_arm_controllers/commands',30) 
# /mars_car_controllers/commands
# /mars_car_wheel_controllers/commands

    def listener_callback(self,joy):

            drive.data[0]=joy.axes[0]
            drive.data[1]=-joy.axes[0]
            drive.data[2]= joy.axes[0]
            drive.data[3]= -joy.axes[0]
            vel.data[0]= joy.axes[4]
            vel.data[1]= joy.axes[4]
            vel.data[2]= joy.axes[4]
            vel.data[3]= joy.axes[4]
            vel.data[4]= joy.axes[4]
            vel.data[5]= joy.axes[4]
            self.publisher_drive.publish(drive)  
            self.publisher_vel.publish(vel)  
            print("controling...",drive.data) 
def main(args=None):
    rclpy.init(args=args)
    control = Control()
    rclpy.spin(control)
    rclpy.shutdown()

    
    
if __name__ == '__main__':
    main()
