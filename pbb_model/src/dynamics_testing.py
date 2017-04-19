
import dynamic_model
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
needle = dynamic_model.DynamicModel()

n = needle.update(.005,.5,.1)
for i in xrange(300):

    n =  np.hstack( (n,needle.update(.005,0,.1)))

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(n[0,1:], n[1,1:], n[2,1:], label='parametric curve')
ax.set_xlim([-0.02,0.02])
ax.set_ylim([-0.02,0.02])
ax.set_zlim([0,0.2])
ax.legend()

plt.show()
