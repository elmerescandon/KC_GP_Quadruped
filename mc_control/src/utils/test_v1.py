import fr_utils as fr 
import numpy as np
from mc_legs import *


q = np.array([-np.pi/6,-np.pi/6,0])

T = fk_robot(q,'FR','pos')
print(T)
# Q = rot2quaternion(T[0:3,0:3])
# Q_b = TQb(Q)


# J_a= Jan_leg(q,'FR')
# J_g = Jgeom_leg(q,'FR')


# E_1 = np.vstack((np.eye(3), np.zeros((3,3)))) 
# E_2 = np.vstack((np.zeros((3,4)),Q_b))
# E = np.hstack((E_1,E_2))

# Twist = E.dot(J_a)


# print("Task Jacobian")
# print(np.round(J_a,3))
# print("Geometric Jacobian")
# print(np.round(J_g,3))
# print("Analytica to Jacobian throught Qb matrix")
# print(np.round(Twist,3))
# print(np.round(E,4))
