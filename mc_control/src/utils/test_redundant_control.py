import numpy as np 

from mc_robot import mini_cheetah
from mc_legs import * 
from copy import copy
from cvxopt import solvers
from cvxopt import matrix


from qpoases import PyQProblemB as QProblemB
from qpoases import PyOptions as Options
from qpoases import PyPrintLevel as PrintLevel


# Implementation of the task-based redundant control
# Based on the Advanced Robotics - Redundant Control class



# Load Robot state

robot = mini_cheetah() 
pose_base = np.array([0,0,0.4424,1,0,0,0])
joints_legs = np.zeros((12))

q_robot = np.concatenate((pose_base, joints_legs))
robot.step_update(q_robot)

# Obtain actual position
pos_fr = robot.leg_position('FR',type='pos')
pos_rr = robot.leg_position('RR',type='pos')

# print(pos_rr)

# Obtain desired positions for each leg only two tasks 
posd_fr = copy(pos_fr)
posd_fr[0] += 0.1
posd_fr[1] -= -0.12

posd_rr = copy(pos_rr)
posd_rr[2] += 0.1
posd_rr[1] -= 0.15



# ================================================
# Weighted task method - Kinematic Control
# ================================================

# Lambdas  (K control)
l_1 = 0.5
l_2 = 0.5

# Obtain error for tasks 
e_1 = robot.error_position_leg(posd_fr,'FR')
e_2 = robot.error_position_leg(posd_rr,'RR')



# Derivate the error
# The derivative of the error is positive, since the error is calculated inversely (x_des - x) -> e_dot = e*lambda
e_1_dot = l_1*e_1
e_2_dot = l_2*e_2
e_3_dot = np.zeros((3))
e_4_dot = np.zeros((3))

# Weights for each task
w_1 = 1.0 
w_2 = 1.0
w_3 = 1.0
w_4 = 1.0

# Jacobian for each task
J_1 = robot.leg_jacobian_position('FR')
J_2 = robot.leg_jacobian_position('RR')
J_3 = robot.leg_jacobian_position('FL')
J_4 = robot.leg_jacobian_position('RL')


# ========================================
# Create Quadratic programming
# ========================================

P = w_1*(J_1.T).dot(J_1)  + w_2*(J_2.T).dot(J_2) # + w_3*(J_3.T).dot(J_3) + w_4*(J_4.T).dot(J_4)
b = -2*(w_1*(J_1.T).dot(e_1_dot) + w_2*(J_2.T).dot(e_2_dot)) #+ w_3*(J_3.T).dot(e_3_dot) + w_4*(J_4.T).dot(e_4_dot))

qmin = np.array([-1.4, -1.4, -2.5,
                 -1.4, -1.4, -2.5, 
                 -1.4, -1.4, -2.5,
                 -1.4, -1.4, -2.5])
qmax = -qmin


dqmax = 10.0*np.ones((12))
dqmin = -dqmax

# Bounds for the floating base
low = -1e1
high = 1e1
lfb = np.array([low, low, low, low, low, low, low])
ufb = np.array([high, high, high, high, high, high, high])
        
dt = 0.01

lower_limits = np.maximum((qmin-q_robot[7:])/dt, dqmin)
upper_limits = np.minimum((qmax-q_robot[7:])/dt, dqmax)

lower_limits = np.hstack((lfb, lower_limits))
upper_limits = np.hstack((ufb, upper_limits))




# Solver
solver = QProblemB(19)
options = Options()
options.setToMPC()
options.printLevel = PrintLevel.LOW
solver.setOptions(options)

nWSR = np.array([10])
solver.init(P, b, lower_limits, upper_limits, nWSR)
dq = np.zeros(19)
solver.getPrimalSolution(dq)
print(dq)
# G = matrix(np.vstack((-np.eye(19),np.eye(19))))
# h = matrix(limits)

# P = matrix(P)
# b = matrix(b)

# # Solve the quadratic problem
# sol = solvers.qp(P,b,G=G,h=h)
# print(P)
# print(b)
# print(sol)
# print(sol['x'])
# print(sol['primal objective'])
# print(sol['status'])

# a = np.array(sol['x'])
# a = np.resize(a,(19,))
# print(a)