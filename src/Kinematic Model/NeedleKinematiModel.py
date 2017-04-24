# Webster Non-Holonomic model
import math
import numpy as np
from scipy.linalg import expm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#Metric Units
u1 = 0.001
u2 = 0

l1  = 0.04; #Distance B to C
phi = 0.17767451785
l2 = 0.023775
k = math.tan(phi)/l1
r = 1/k
e1 = np.array([[1],[0],[0]])
e3 = np.array([[0],[0],[1]])
V1 = np.concatenate(([[e3],[k*e1]]), axis=0)
V2 = np.concatenate(([[0],[0],[0]],e3), axis=0)

def isFeasible(x,y,z):
    d = math.sqrt(math.pow(x,2)+math.pow(y,2))
    if (math.pow(d,2)<math.pow(r,2)):
    	z_tip = math.sqrt(math.pow(r,2)-math.pow(d,2))
    else:
	return True
    if (z_tip<z):
        return True
    return False

def kinematicModel(u1,u2,T,state):
    e1 = np.array([[1],[0],[0]])
    e3 = np.array([[0],[0],[1]])
    V1 = np.concatenate(([[e3],[k*e1]]), axis=0)
    V2 = np.concatenate(([[0],[0],[0]],e3), axis=0)
    V1_hat = np.concatenate((isomorphic(V1[1]),V1[0]), axis=1)
    V1_hat = np.vstack((V1_hat,np.array([0,0,0,0])))
    V2_hat = np.concatenate((isomorphic(V2[3:6]),V2[0:3]), axis=1)
    V2_hat = np.vstack((V2_hat,np.array([0,0,0,0])))
    
    new_state = np.array(state[:,:].dot(expm((u1*V1_hat + u2*V2_hat)*T)))
    n = (new_state[0:3,0:3]*l2).dot(e3)+(new_state[0:3,3]).reshape(3,1)

    return (n, new_state)

def planPath(x,y,z):
    path = []
    if (isFeasible(x,y,z) is False):
	print "Not Feasible"
	return False
    else:
	print "Feasible"
	#Calculate initial angle
	angle = math.atan2(y,x)
        #Create initial state
	state = np.array([[math.cos(angle+math.pi/2),-math.sin(angle+math.pi/2),0,0],
			  [math.sin(angle+math.pi/2),math.cos(angle+math.pi/2),0,0],			
			  [0,	0,	1,	0],
			  [0,	0,	0,	1]])

	d_c = math.sqrt(math.pow(abs(x)-r,2)+math.pow(abs(y)-r,2)+math.pow(z,2))
	print d_c
	d_thres = math.sqrt(math.pow(d_c,2)-math.pow(r,2))
	t = 0.02
	distance = 1000000

	while (distance > d_thres):
	    (n,state) = kinematicModel(0.01,0,t,state)
	    path.append(n)
	    distance = math.sqrt(math.pow(n[0]-x,2)+math.pow(n[1]-y,2)+math.pow(n[2]-z,2))
	    
	path.append(np.array([[x],[y],[z]]))
	return path

def isomorphic(x):
    return np.array([[0,-x.item(2),x.item(1)],
		[x.item(2),0,-x.item(0)],			
		[-x.item(1),x.item(0),0]])

V1_hat = np.concatenate((isomorphic(V1[1]),V1[0]), axis=1)
V1_hat = np.vstack((V1_hat,np.array([0,0,0,0])))
V2_hat = np.concatenate((isomorphic(V2[3:6]),V2[0:3]), axis=1)
V2_hat = np.vstack((V2_hat,np.array([0,0,0,0])))

#T = 0.1; # seconds per step 

#g_ab = np.array([[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]])

#n = np.zeros((3,1))

q = planPath(-0.4,-0.4,0.6)
#for i in range (0,3000):
#    g_ab = np.vstack((g_ab,np.array([(g_ab[i,:,:]).dot(expm((u1*V1_hat + u2*V2_hat)*T))])))
#    n = np.hstack((n,(g_ab[i,0:3,0:3]*l2).dot(e3)+(g_ab[i,0:3,3]).reshape(3,1)))
 
#print isFeasible(0,4,0.2)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

hoe = np.zeros((3,1))
for i in q:
    hoe = np.hstack((hoe,i))
    
print hoe
ax.plot(hoe[0][1:], hoe[1][1:], hoe[2][1:], label='parametric curve')
plt.xlabel('x')
plt.ylabel('y')
ax.legend()
plt.xlim(-0.5, 0.5)
plt.ylim(-0.5, 0.5)

plt.show()
	
