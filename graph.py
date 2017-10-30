
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as manimation

FFMpegWriter = manimation.writers['ffmpeg']
metadata = dict(title='Movie Test', artist='Matplotlib',
                comment='Movie support!')
writer = FFMpegWriter(fps=10, metadata=metadata)

fig = plt.figure()

data = np.loadtxt('data.txt')
print(data)
a = np.array([[0, 0, 0, 0, 0, -1]])


X, Y, Z, U, V, W = zip(*a)
ax = fig.add_subplot(111, projection='3d')
vec = ax.quiver(X, Y, Z, U, V, W)
ax.set_xlim([-5, 5])
ax.set_ylim([-5, 5])
ax.set_zlim([-5, 5])
row = 0

with writer.saving(fig, "gyro.mp4", 100):
	while row < 100:
		a = np.array([[0, 0, 0, data[row][3], data[row][4], data[row][5]]])
		X, Y, Z, U, V, W = zip(*a)
		ax.cla()
		vec = ax.quiver(X, Y, Z, U, V, W)
		ax.set_xlim([-1, 1])
		ax.set_ylim([-1, 1])
		ax.set_zlim([-1, 1])
		writer.grab_frame()
		plt.pause(0.1)
		row = row + 1