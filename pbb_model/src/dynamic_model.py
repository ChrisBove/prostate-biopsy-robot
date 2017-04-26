#!/usr/bin/env python
"""@package docstring
    This is the dynamic model of the needle
"""
import copy
import math as m
import numpy as np

from scipy.linalg import expm


class DynamicModel(object):
    """ fo dynamic_model."""

    def __init__(self):

        self._l = 4

        self._N = 4
        self._P = np.asmatrix( (2.0 / self._l) * np.ones((self._N, 1)))
        self._C0 = np.asmatrix(np.ones((1, self._N)))
        np.put(self._C0, [0], [0.5])
        #(self._P, self._Q ) = self.get_needle_forces()

        self._G = 1.841210  # shear mo(dulus
        self._J = 1.5962 * 10**(-14)  # polar momemoment Interia
        self._pho = 6.453
        self._b = 10  # coef of friction
        self._beta = 4.753 * 10**(-3)
        self._kappa = 1.3698630137  # kappa

        self._Cl = np.ones((1, self._N))
        for ii in xrange(self._N):
            np.put(self._Cl, [ii], [(-1)**ii])
        np.put(self._Cl, [0], [.5])
        # updated every step
        self._depth = 0
        self._s_state =  np.array( [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        self._q_state = np.asmatrix(np.zeros((self._N, 1)))
        self._n = np.zeros((3,1))

    def update(self, u1, u2, dt):

        """Updates the systems"""

        # check if inputs are 0
        if (u1 != 0.0 and u2 != 0.0): 
            print "I didn't get zeros"
            self.update_depth(u1, dt)

            A = self.get_Amat(u1)
            B = self.get_Bmat(u1)
            C = self.get_Cmat(A)
            D = self.get_Dmat(B)


            (self._n, self._s_state) = self.update_S(u1, u2, C, D, dt)

            self._q_state = A * self._q_state + B * u2
        #print self._q_state
        return self._n

    def update_depth(self, v, dt):
        """This updates the insertion depth"""
        self._depth = self._depth + v * dt

    def update_S(self, u1, u2, C, D, dt):
        """
            Update the S state
            u1 is linear velolicy
            u2 is the angular velocity

           """
        l2 =  0.023775
        state = self._s_state
        # this are the unit vectors to move it into the right frame
        e1 = np.array([[1],[0],[0]])
        e3 = np.array([[0],[0],[1]])
        V1 = np.concatenate(([[e3],[self._kappa*e1]]), axis=0)
        V2 = np.concatenate(([[0],[0],[0]],e3), axis=0)
        V1_hat = np.concatenate((self.isomorphic(V1[1]),V1[0]), axis=1)
        V1_hat = np.vstack((V1_hat,np.array([0,0,0,0])))
        V2_hat = np.concatenate((self.isomorphic(V2[3:6]),V2[0:3]), axis=1)
        V2_hat = np.vstack((V2_hat,np.array([0,0,0,0])))
        compinsation  =  np.asscalar (C*self._q_state + D*u2)
        #print  "real input ",  compinsation
        new_state = state[:, :].dot(expm(( u1*V1_hat +   compinsation*V2_hat) * dt))
        temp = np.array([new_state])
        n = ((temp[0, 0:3, 0:3] * l2).dot(e3) + (temp[0, 0:3, 3]).reshape(3, 1))

        return (n, new_state)

    def isomorphic(self, x):
        return np.array([[0, -x.item(2), x.item(1)],
                         [x.item(2), 0, -x.item(0)],
                         [-x.item(1), x.item(0), 0]])

    def get_Amat(self, v):
        """This function calculate the A matrix"""

        invD = self.get_invD(v)
        K = self.get_K(v)
        # this is link a sring constant
        lumped_sping = (self._J * self._G) / (self._l - self._depth)
        makeMatrix = K + lumped_sping * self._P*self._C0

        A = -invD*makeMatrix
        return A

    def get_Bmat(self, v):
        """This function calculate the
        B matrix"""
        invD = self.get_invD(v)
        # this is link a sring constant
        lumped_sping = (self._J * self._G) / (self._l - self._depth)

        B = invD*self._P*lumped_sping
        return B

    def get_Cmat(self, A):
        """This function calculate the C matrix"""
        return self._Cl*A

    def get_Dmat(self, B):
        """This function returns the D matrix"""
        return self._Cl*B

    def get_invD(self,  v):
        """get the D matrix"""

        d = self.get_d_coef(v)

        D = np.zeros((self._N, self._N))
        coef = (self._J * self._pho * v) / self._depth

        for i in xrange(self._N):
            for j in xrange(self._N):
                D[i][j] = coef * d[i][j]

        return np.linalg.pinv(np.asmatrix(D))

    def get_d_coef(self, v):
        """This function get the little d coef """

        d = [[0 for i in xrange(self._N)] for i in xrange(self._N)]
        for i in xrange(1, self._N):
            for j in xrange(1, self._N):

                if (j > i):
                    d[i - 1][j - 1] = (-4 * ((-1)**(i - j))
                                       * (j - 1)**2) / ((j - i) * (i + j - 2))
                elif (i == j):
                    d[i - 1][j - 1] = np.sign(1 - i) + \
                        (self._beta * self._depth) / (self._J * self._pho * v)
                elif (j < i):
                    d[i - 1][j - 1] = (4 * ((-1)**(j - i))
                                       * (j - 1)**2) / ((i - j) * (i + j - 2))

        return d

    def get_K(self, v):
        """get the K matrix"""

        k = self.get_k_coef(v)
        K = np.zeros((self._N, self._N))
        coef = (self._J * self._pho * v * v) / (self._depth**2)

        for i in xrange(self._N):
            for j in xrange(self._N):
                K[i][j] = coef * k[i][j]
        return np.asmatrix(K)

    def get_k_coef(self, v):
        """This function get the little k coef """
        k = [[0 for i in xrange(self._N)] for i in xrange(self._N)]
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
        J = np.matrix([[m.cos(beta) * m.cos(gamma), m.sin(gamma), 0],
                       [-m.cos(beta) * m.sin(gamma), m.cos(gamma), 0],
                       [m.sin(beta), 0, 1]])
        return J

    def reset(self):
        self._depth = 0
        self._s_state =  np.array( [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        self._q_state = np.asmatrix(np.zeros((self._N, 1)))
        self._n = np.zeros((3,1))

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
