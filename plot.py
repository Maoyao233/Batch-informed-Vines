from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import sys


data = np.loadtxt('path.txt')

n = int(sys.argv[1])
r = float(sys.argv[2])


fig, ax = plt.subplots()
x_ticks = np.arange(0, n+1, 1)
y_ticks = np.arange(0, n+1, 1)
ax.set_xticks(x_ticks)
ax.set_yticks(y_ticks)
ax.grid(True, which='both')

for i in x_ticks:
    for j in y_ticks:
        circle = plt.Circle((i, j), r, fill=False)
        ax.add_patch(circle)

plt.xlim(0, n)
plt.ylim(0, n)
plt.gca().set_aspect('equal', adjustable='box')

ax.plot(data[:,0],data[:,1],'.-')
plt.show()