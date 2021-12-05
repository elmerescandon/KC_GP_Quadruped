import fr_utils as fr
import numpy as np
from mc_legs import *
from mc_robot import *

# q = np.array([0,0,0])
# T = fk_robot(q,'RL')
# print(np.round(T,3))
# Q = np.array([1,0,0,0])


Z = 0.4424
q_0 = np.array([0., 0., 0.4424, 
                1., 0., 0., 0.,
                0,0,0, 0,0,0, 0,0,0, 0,0,0])

robot = mini_cheetah()
robot.step_update(q_0)

pose_leg = robot.leg_position('FR','full')
J_leg = robot.leg_jacobian('FR')
print(np.round(pose_leg,4))
print(np.round(J_leg,4))



# qd = np.array([np.pi/6,np.pi/5,0])
# Td = fk_robot(qd,'RL')
# Qd = rot2quaternion(Td[0:3,0:3])

# Qe = diffQuat(Q,Qd)
# print(Qe)
# print(Qe[0]**2 + Qe[1]**2 + Qe[2]**2 + Qe[3]**2 )