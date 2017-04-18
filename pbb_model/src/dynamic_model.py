#!/usr/bin/env python
"""@package docstring
    This is the dynamic model of the needle
"""
import rospy
import copy
import math as m
import numpy as np
from std_msgs.msg import String


class DynamicModel(object):
    """docstring fo dynamic_model."""

    def __init__(self):

        self._l = 10
        self._Cl = []
        self._P =  np.asmatrix( 2 / l * np.ones((self._N, 1)))
        self._C0 = np.asmatrix(  np.ones((1, self._N)))
        np.put(self._C0,[0],[0.5])
        #(self._P, self._Q ) = self.get_needle_forces()
        self._N = 25
        self._G = 1.841210  # shear mo(dulus
        self._J = 1.5962 * 10**(-14)  # polar momemoment Interia
        self._pho = 6.453
        self._b = 10  # coef of friction
        self._beta = 4.753 * 10**(-3)
        self._radius = 13.698630137  # kappa

        self._Cl =  np.ones((1, self._N))
        for ii in xrange(25):
	        np.put(self._Cl, [ii], [ (-1)**ii ] )
        np.put(self._Cl, [0], [ .5 ] )



    def dynamics(self, u1, u2):
        e1 = np.array([[1],[0],[0]])
        e3 = np.array([[0],[0],[1]])
        V1 = np.concatenate(([[e3],[k*e1]]), axis=0)
        V2 = np.concatenate(([[0],[0],[0]],e3), axis=0)

        sdot

    def update_l(self, l,v ):

        dt = .1
        return l + v*dt

    def get_Amat(self, l, v):
        """This function calculate the A matrix"""

        invD = self.get_invD(l.v)
        K = self.get_K(l, v)
        # this is link a sring constant
        lumped_sping = (self._J * self._G) / (self._l - l)
        makeMatrix = K + lumped_sping * self._P * self._C0
        A = -invD * makeMatrix
        return A

    def get_Bmat(self,l,v):
        """This function calculate the B matrix"""
        invD = self.get_invD(l.v)
        # this is link a sring constant
        lumped_sping = (self._J * self._G) / (self._l - l)
        B = invD * lumped_sping * self._P
        return B

    def get_Cmat(self, A):
        """This function calculate the C matrix"""
        return self._Cl*A

    def get_Dmat(self, B):
        """This function returns the D matrix"""
        return self._Cl*B

    def get_invD(self, l, v):
        """get the D matrix"""

        d = self.get_d_coef(l, v):
        D = numpy.zeros((self._N, self._N))
        coef = (self._J * self._pho * v) / l

        for i in xrange(1, self_N):
            for j in xrange(1, self_N):
                D[i][j] = coef * d[i][j]
        return np.linalg.inv(np.asmatrix(D))

    def get_d_coef(self, l, v):
        """This function get the little d coef """

        d = [][]
        for i in xrange(1, self._N):
            for j in xrange(1, self._N):

                if (j > i):
                    d[i - 1][j - 1] = (-4 * ((-1)**(i - j))
                                       * (j - 1)**2) / ((j - i) * (i + j - 2))
                elif (i == j):
                    d[i - 1][j - 1] = np.sign(1 - i) + \
                        (self._beta * l) / (self._J * self._pho * v)
                elif (j < i):
                    d[i - 1][j - 1] = (4 * ((-1)**(j - i))
                                       * (j - 1)**2) / ((i - j) * (i + j - 2))
        return d

    def get_K(self, l, v):
        """get the K matrix"""

        d = self.get_k_coef(l):
        K = numpy.zeros((self._N, self._N))
        coef = (self._J * self._pho * v * v) / (l * l)

        for i in xrange(1, self_N):
            for j in xrange(1, self_N):
                K[i][j] = coef * d[i][j]
        return np.asmatrix( K )

    def get_k_coef(self, l):
        """This function get the little k coef """
        k = [][]
        for i in xrange(1, self._N):
            for j in xrange(1, self._N):
                if (i == 0 or j == 0):
                    k[i - 1][j - 1] = 0
                elif (i == j):
                    k[i - 1][j - 1] = ((2.0 * np.pi * self._G * (i - 1)**2) /
                                       (self._pho * v * v)) - np.pi * np.pi * (i - 1) / 3.0 + 0.5
                elif (j != i):
                    k[i - 1][j - 1] = (-8 * (i - 1)**2 * (j - 1)**2 *
                                       (-1)**(i - j)) / ((j - i)**2 * (i + j - 2)**2)
        return k


    def get_jacobian(self, beta, gamma):
        """calculates the Jacobian"""
        J = np.matrix( [ [ m.cos(beta)*m.cos(gamma), m.sin(gamma), 0 ], \
                         [ -m.cos(beta)*m.sin(gamma), m.cos(gamma), 0], \
                         [ m.sin(beta), 0, 1 ]])
        return J


    def get_needle_forces(self):
        """
            Calulates the force acting on the needle tip
            returns P and Q
        """
        lamada = np.pi / 6  # bevel angle
        a = 1  # tip lenght
        b = 1  # bevel length
        Kt = 10  # material property
        beta = .3 * lamada  # acting angle

        T = 0.5 * Kt * b * b

        alpha = (m.tan(lamada) - m.tan(beta)) / \
            (1 + m.tan(lamada) * m.tan(beta))

        # see diagram for force
        P = T * m.sin(lamada * alpha)
        Q = T * m.cos(lamada * alpha) - 0.5 * Kt * a * a * m.tan(beta)

        return (P, Q)


if __name__ == '__main__':
    rospy.init_node('dynamic_model', anonymous=True)
