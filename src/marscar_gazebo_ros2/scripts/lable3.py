import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# 定义刚体的形状，这里假设是一个简单的立方体
def create_cube(ax):
    # 创建立方体的顶点数据
    cube_verts = [
        (1, 1, 1), (-1, 1, 1), (-1, -1, 1), (1, -1, 1),
        (1, 1, -1), (-1, 1, -1), (-1, -1, -1), (1, -1, -1)
    ]
    edges = [(0, 1), (0, 3), (0, 4), (2, 1), (2, 3), (2, 7),
             (6, 3), (6, 4), (6, 7), (5, 1), (5, 4), (5, 7)]
    
    # 绘制立方体
    ax.plot(np.array(cube_verts).T[:, 0],
            np.array(cube_verts).T[:, 1],
            np.array(cube_verts).T[:, 2], 'r')
    for edge in edges:
        ax.plot([cube_verts[edge[0]][0], cube_verts[edge[1]][0]],
                [cube_verts[edge[0]][1], cube_verts[edge[1]][1]],
                [cube_verts[edge[0]][2], cube_verts[edge[1]][2]], 'k')

# 初始化图形窗口
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 创建立方体并添加到图形中
create_cube(ax)

# 定义旋转函数，这里只考虑绕Z轴旋转为例
def rotate_z(angle):
    ax.view_init(elev=ax.elev, azim=angle)

# 设置动画帧数
num_frames = 360  # 一圈360度

# 创建动画函数
def animate(i):
    angle = i
    rotate_z(angle)
    return fig,

# 创建动画
ani = animation.FuncAnimation(fig, animate, frames=num_frames,
                              interval=20, blit=False, repeat=True)

# 显示动画
plt.show()