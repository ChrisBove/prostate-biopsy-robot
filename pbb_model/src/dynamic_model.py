#!/usr/bin/env python
# license removed for brevity
import rospy
import math as m
import numpy as np
from std_msgs.msg import String


class DynamicModel(object):
    """docstring fo dynamic_model."""
    def __init__(self, M, B, K):

        self._M = M
        self._B = B
        self._K = K
        (self._P, self._Q ) = self.get_needle_forces()

    def inverseModel(self, ):
        return


    """
        Calulates the force acting on the needle tip
        returns P and Q
    """
    def get_needle_forces(self):

        lamada = np.pi/6 # bevel angle
        a = 1 # tip lenght
        b = 1 # bevel length
        Kt = 10 # material property
        beta = .3*lamada # acting angle

        T = 0.5*Kt*b*b

        alpha = ( m.tan(lamada) - m.tan(beta)) / ( 1 + m.tan(lamada)*m.tan(beta))
        # force acting paralle with the driving force of the needle
        P = T *m.sin(lamada*alpha )

        Q = T*m.cos(lamada*alpha) - 0.5*Kt*a*a*m.tan(beta)

        return (P, Q)





        pass

if  __name__ == '__main__':
    rospy.init_node('dynamic_model', anonymous=True)
