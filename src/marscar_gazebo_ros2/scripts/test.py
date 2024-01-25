import numpy as np

from scipy import interpolate

x = np.arange(-5.01, 5.01, 0.25)

y = np.arange(-5.01, 5.01, 0.25)

xx, yy = np.meshgrid(x, y)

z = np.sin(xx**2+yy**2)

f = interpolate.interp2d(x, y, z, kind='cubic')

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
fig = plt.gcf()
xnew = np.arange(-5.01, 5.01, 0.01)

ynew = np.arange(-5.01, 5.01, 0.01)

znew = f(xnew, ynew)
ax = fig.gca(projection='3d')        
ax.scatter3D(xnew, ynew,znew[0, :],'blue')
# plt.plot(x, z[0, :], 'ro-', xnew, znew[0, :], 'b-')
print(len(xnew))
plt.show()
