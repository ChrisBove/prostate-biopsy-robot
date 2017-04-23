
import dynamic_model
import numpy as np
import matplotlib.pyplot as plt
import random
from mpl_toolkits.mplot3d import Axes3D
needle = dynamic_model.DynamicModel()

n = needle.update( 0.001 , 0 ,1)
for i in xrange(1000):

    n =  np.hstack( (n,needle.update(.001 , 0,1)))

fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(n[0,1:], n[1,1:], n[2,1:])

ax.legend()

plt.show()
