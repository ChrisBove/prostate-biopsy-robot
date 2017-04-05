# Webster Non-Holonomic model
import math
import numpy as np
from scipy.linalg import expm

def isomorphic(x):
    return np.array([[0,-x.item(2),x.item(1)],
		[x.item(2),0,-x.item(0)],			
		[-x.item(1),x.item(0),0]])
	
def kinematicModel(u1,u2,T,state):
    l1  = 0.04; #Distance B to C
    phi = 0.17767451785
    l2 = 0.023775
    k = math.tan(phi)/l1
    e1 = np.array([[1],[0],[0]])
    e3 = np.array([[0],[0],[1]])
    V1 = np.concatenate(([[e3],[k*e1]]), axis=0)
    V2 = np.concatenate(([[0],[0],[0]],e3), axis=0)
    V1_hat = np.concatenate((isomorphic(V1[1]),V1[0]), axis=1)
    V1_hat = np.vstack((V1_hat,np.array([0,0,0,0])))
    V2_hat = np.concatenate((isomorphic(V2[3:6]),V2[0:3]), axis=1)
    V2_hat = np.vstack((V2_hat,np.array([0,0,0,0])))
    
    new_state = np.array([state[:,:].dot(expm((u1*V1_hat + u2*V2_hat)*T))])
    n = ((new_state[0,0:3,0:3]*l2).dot(e3)+(new_state[0,0:3,3]).reshape(3,1))

    return (n, new_state)




