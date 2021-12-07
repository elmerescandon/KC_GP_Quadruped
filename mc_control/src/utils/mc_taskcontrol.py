# Import libraries
import numpy as np 
from copy import copy, error
from cvxopt import solvers
from cvxopt import matrix
from cvxopt import spmatrix


# Create object controller only for legs

class mc_controller: 
    def __init__(self,lambdas, weights,dt,type_c='legs'):
        self.type_c = type_c
        self.l = lambdas
        self.w = weights
        self.dt = dt
        # Motor Limits
        self.qmax = np.array([1.04,3.92,-0.61,
                         1.04,3.92,-0.61,
                         1.04,3.92,-0.61,
                         1.04,3.92,-0.61])
        
        self.qmin = np.array([-0.87,-0.52,-2.77,
                              -0.87,-0.52,-2.77,
                              -0.87,-0.52,-2.77,
                              -0.87,-0.52,-2.77])

        self.dqmax = 10.6*np.ones((12)) # Velocity Limits
        self.dqmin = -self.dqmax
        # Floating base limits
        self.base_limits = 1e8

        self.base_max = self.base_limits*np.ones((7))
        self.base_min = -self.base_limits*np.ones((7))


        # Matrices for Quadratic Programming
        self.G = spmatrix([1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
                           -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, -1,-1,-1],
                          [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37],
                          [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18])

    def apply_control(self,errors,J_tsk,q_robot): 

        # Indicate limits
        min_jointslimit = np.maximum((self.qmin- q_robot[7:])/self.dt, self.dqmin)
        max_jointslimit = np.minimum((self.qmax- q_robot[7:])/self.dt, self.dqmax)

        lower_limits = np.hstack((self.base_min, min_jointslimit))
        upper_limits = np.hstack((self.base_max, max_jointslimit))

        # Lower limits must be on negative to follow the inequality rule
        h = matrix(np.hstack((upper_limits,-lower_limits)))

        # Construct sum of matrices based on Weighted task method
        w_1 = self.w[0]
        w_2 = self.w[1]
        w_3 = self.w[2]
        w_4 = self.w[3]

        J_1 = J_tsk[0]
        J_2 = J_tsk[1]
        J_3 = J_tsk[2]
        J_4 = J_tsk[3]

        e_1_dot = self.l[0]*errors[0]
        e_2_dot = self.l[1]*errors[1]
        e_3_dot = self.l[2]*errors[2]
        e_4_dot = self.l[3]*errors[3]

        P = matrix(w_1*(J_1.T).dot(J_1)  + w_2*(J_2.T).dot(J_2) + w_3*(J_3.T).dot(J_3) + w_4*(J_4.T).dot(J_4))
        b = matrix(-2*(w_1*(J_1.T).dot(e_1_dot) + w_2*(J_2.T).dot(e_2_dot) + w_3*(J_3.T).dot(e_3_dot) + w_4*(J_4.T).dot(e_4_dot)))

        if self.type_c == 'base': 
            w_b = self.w[4]
            J_b = J_tsk[4]
            e_b_dot = self.l[4]*errors[4]

            P = P + matrix(w_b*(J_b.T).dot(J_b))
            b = b + matrix(-2*w_b*(J_b.T).dot(e_b_dot))

        # Apply function to solve by QP
        solvers.options['show_progress'] = 0
        sol = solvers.qp(P,b,G=self.G,h=h)

        q_dot = sol['x']
        q_dot = np.array(q_dot)
        q_dot = np.resize(q_dot,(19,))
        return q_dot



    