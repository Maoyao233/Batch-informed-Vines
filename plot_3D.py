from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt
data = numpy.loadtxt('path.txt')
fig = plt.figure()
ax = fig.add_axes(Axes3D(fig))
ax.plot(data[:,0],data[:,1],data[:,2],'.-')
plt.show()