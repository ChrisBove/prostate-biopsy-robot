#!/usr/bin/env python
# license removed for brevity
import rospy
import math as m
import numpy as np
from std_msgs.msg import String


class DynamicModel(object):
    """docstring fo dynamic_model."""
    def __init__(self):

        self._l = 10
        self._Cl = [ ]
        self._P = 2/l*np.array([1,1,1,1,1,1])
        #(self._P, self._Q ) = self.get_needle_forces()
        self._N = 25
        self._G = 10^6 #shear modulus
        self._J = 100 #polar moment Interia
        self._pho = 10


    def get_d_coef(self,l,v,beta):
        """This function get the little d coef """
        d = [][]

        for i in xrange(1,self._N):
            for j in xrange(1,self._N):

                if (j > i):
                    d[i-1][j-1] = ( -4* ((-1)**(i-j))*(j-1)**2)/ ( ( j-i )*( i + j - 2))
                elif ( i == j ):
                    d[i-1][j-1] = np.sign( 1 - i) + (beta*l)/(self._J*self._pho*v)
                elif ( j < i):
                    d[i-1][j-1] = ( 4* ((-1)**(j-i))*(j-1)**2)/ ((i -j)* ( i + j - 2))
        return d



    def get_k_coef(self, ):
        k = [][]
        for i in xrange(1,self._N):
            for j in xrange(1,self._N):
                if (i == 0 or j ==0):
                    k[i-1][j-1] = 0
                elif ( i == j ):
                    k[i-1][j-1] = np.sign( 1 - i) + (beta*l)/(self._J*self._pho*v)
                elif ( j < i):
                    k[i-1][j-1] = ( 4* ((-1)**(j-i))*(j-1)**2)/ ((i -j)* ( i + j - 2))




    def get_needle_forces(self):
        """
            Calulates the force acting on the needle tip
            returns P and Q
        """

        lamada = np.pi/6 # bevel angle
        a = 1 # tip lenght
        b = 1 # bevel length
        Kt = 10 # material property
        beta = .3*lamada # acting angle

        T = 0.5*Kt*b*b

        alpha = ( m.tan(lamada) - m.tan(beta)) / ( 1 + m.tan(lamada)*m.tan(beta))

        # see diagram for force
        P = T *m.sin(lamada*alpha )
        Q = T*m.cos(lamada*alpha) - 0.5*Kt*a*a*m.tan(beta)

        return (P, Q)


if  __name__ == '__main__':
    rospy.init_node('dynamic_model', anonymous=True)
