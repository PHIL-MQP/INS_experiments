'''
parse the ins_data file, get each of the axes and a timestamp into a np.matrix
'''

import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as manimation

f = open('ins_data.txt')
temp = np.asarray(f.read().splitlines())
temp2 = []
for i in temp:
	temp2.append(np.asarray(i.split(' ')))

mat = np.asmatrix(temp2)
for i in range(0, mat.shape[0]-1):
	d = (int(mat[i+1,6])*1000000+int(mat[i+1,7])) - (int(mat[i,6])*1000000+int(mat[i,7]))
	if(d > 10500):
		print( d, ' ',i )

print(mat.shape)
# data = np.asmatrix(temp)

# print(data)
# print(temp,end='\n')
