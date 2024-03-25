import matplotlib.pyplot as plt
import numpy as np
# 定义抛物线函数
def parabolic_function(x, a, b, c):
    return a * x**2 + b * x + c

# 抛物线的系数
a = 1.0
b = 0.0
c = 0.0


# 假设已经有了力F、速度V、位置X随时间t变化的数据，同时设定时间步长dt
# 例如：
t = np.linspace(0, 10, 1000)  # 时间数组，从0到10秒，共1000个时间点
dt = t[1] - t[0]  # 计算时间间隔
F = parabolic_function(t, a, b, c)  # 力F随时间的变化数据，需要与时间t长度相同
V = parabolic_function(t, a, b, c)  # 速度V随时间的变化数据，同样长度
X = parabolic_function(t, a, b, c)  # 位置X随时间的变化数据，同样长度

fig, ax = plt.subplots()

# 绘制力F、速度V和位置X随时间变化的曲线
ax.plot(t, F, label='force(F)')
ax.plot(t, V, label='velocity(V)')
ax.plot(t, X, label='position(X)')

# 在每个时间间隔处添加标记，这里仅示例性标注前几个点
for i in range(0, len(t), int(1/dt)):  # 根据dt选择要标注的点的数量
    ax.annotate(f'({"F"+str(i)}, {"V"+str(i)}, {"X"+str(i)})', 
                xy=(t[i], F[i]), xytext=(-20, 20), textcoords='offset points',
                arrowprops=dict(facecolor='black', shrink=0.05))

ax.set_xlabel('(t)')
ax.set_ylabel('')
ax.legend()
ax.grid(True)

plt.show()