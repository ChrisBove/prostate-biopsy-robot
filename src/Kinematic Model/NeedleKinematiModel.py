# Webster Non-Holonomic model
import math
import numpy as np
from scipy.linalg import expm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#Metric Units
u1 = 0.005
u2 = 0.5

l1  = 0.04; #Distance B to C
phi = 0.17767451785
l2 = 0.023775
k = math.tan(phi)/l1
e1 = np.array([[1],[0],[0]])
e3 = np.array([[0],[0],[1]])
V1 = np.concatenate(([[e3],[k*e1]]), axis=0)
V2 = np.concatenate(([[0],[0],[0]],e3), axis=0)

def isomorphic(x):
    return np.array([[0,-x.item(2),x.item(1)],
		[x.item(2),0,-x.item(0)],			
		[-x.item(1),x.item(0),0]])

V1_hat = np.concatenate((isomorphic(V1[1]),V1[0]), axis=1)
V1_hat = np.vstack((V1_hat,np.array([0,0,0,0])))
V2_hat = np.concatenate((isomorphic(V2[3:6]),V2[0:3]), axis=1)
V2_hat = np.vstack((V2_hat,np.array([0,0,0,0])))

T = 0.1; # seconds per step 

g_ab = np.array([[[1,0,0,0],
        	[0,1,0,0],
        	[0,0,1,0],
        	[0,0,0,1]]])

n = np.zeros((3,1))

for i in range (0,300):
    g_ab = np.vstack((g_ab,np.array([(g_ab[i,:,:]).dot(expm((u1*V1_hat + u2*V2_hat)*T))])))
    n = np.hstack((n,(g_ab[i,0:3,0:3]*l2).dot(e3)+(g_ab[i,0:3,3]).reshape(3,1)))
 
print n[2]
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(n[0,1:], n[1,1:], n[2,1:], label='parametric curve')
ax.set_xlim([-0.02,0.02])
ax.set_ylim([-0.02,0.02])
ax.set_zlim([0,0.2])
ax.legend()

plt.show()
	
