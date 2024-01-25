
import sys
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import ApplyLinkWrench
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.srv import GetEntityState
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Int32,Float64MultiArray
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
# linkattacher_msgs/srv/AttachLink
# gazebo_msgs/srv/ApplyLinkWrench 
# 3.定义节点类；gazebo_msgs/srv/SetEntityState
pos = Float64MultiArray()
pos_x =[]
pos_y =[]
pos_z =[]
plt.ion()
class Pubcable(Node):
    
    def __init__(self):
        super().__init__('cable_plot_py')
        self.subscription = self.create_subscription(LinkStates,'/link_states',self.listener_callback,30)
        self.get_clock().now()
    def listener_callback(self,link_states):
        i=1
        prefix = 'cartpole::sphere'
        for name in link_states.name:
            
            if name.startswith(prefix):
                index=link_states.name.index(name)
                sphere_pose_x=link_states.pose[index].position.x
                sphere_pose_y=link_states.pose[index].position.y
                sphere_pose_z=link_states.pose[index].position.z
                pos_x.append(sphere_pose_x)
                pos_y.append(sphere_pose_y)
                pos_z.append(sphere_pose_z)
                i+=1
        plt.clf()  
        fig = plt.gcf()
        ax_z = Axes3D(fig)
        ax_z.set_zlim((0, 30)) 
        ax_z.set_zlabel("Z", fontsize=14)

        plt.xlim((0, 30))
        plt.ylim((-30, 30))   
        plt.title("cable", fontsize=24) 
        plt.xlabel("X", fontsize=14) 
        plt.ylabel("Y", fontsize=14)
        ax = fig.gca(projection='3d')        
        ax.plot3D(pos_x, pos_y, pos_z,'red')
        # ax.scatter3D(pos_x, pos_y, pos_z,'blue')
        plt.pause(0.001)  
        plt.ioff()  
        
        pos_x.clear()
        pos_y.clear()
        pos_z.clear()  
   
plt.show() 
def main():
    rclpy.init()

    pubcable = Pubcable()
    rclpy.spin(pubcable)



if __name__ == '__main__':
    main()