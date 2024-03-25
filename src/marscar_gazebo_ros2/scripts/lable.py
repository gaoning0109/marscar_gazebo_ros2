import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 物理常数和初始条件
m = 1.0  # 质量
F_net = np.array([10, -20, 30])  # 外力向量
initial_position = np.array([0, 0, 0])  # 初始位置
initial_velocity = np.array([5, 10, 15])  # 初始速度
time_total = 10  # 总时间
delta_t = 0.01  # 时间步长

# 定义欧拉法函数
def euler_step_3d(position, velocity, acceleration, delta_t):
    new_velocity = velocity + acceleration * delta_t
    new_position = position + new_velocity * delta_t
    return new_position, new_velocity

# 初始化存储位置和速度的列表
positions = [initial_position.tolist()]
velocities = [initial_velocity.tolist()]
times = [0]

# 计算加速度
acceleration = F_net / m

# 模拟运动
while times[-1] < time_total:
    # 计算下一个时刻的速度和位置
    position, velocity = euler_step_3d(np.array(positions[-1]), np.array(velocities[-1]), acceleration, delta_t)
    
    # 更新存储的数据
    positions.append(position.tolist())
    velocities.append(velocity.tolist())
    times.append(times[-1] + delta_t)

# 可视化结果
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

ax.plot(*zip(*positions[:-1]), 'o--', color='blue', label='Trajectory')
ax.plot(*zip(*[positions[-1]]), 'ro', markersize=8, label='Final Position')

ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')
ax.set_title('Euler Method')
ax.legend()

plt.show()
