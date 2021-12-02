import fr_utils as fr 
import numpy as np

# Rotation transform from the inertial frame to the robot frame
rot = fr.rot
R1 = rot(90,'y')
R2 = rot(90,'z')
R = R1.dot(R2)
quat = fr.rot2quaternion(R)

# Functions from DH
q = np.array([0,0,0])
A = -0.053565 + 0.02069
# B = 
# C = 

# T01 = dh(q[0], 0, 0, np.pi/2)
# T12 = dh(q[1], -A, B, 0)
# T2EF = dh(q[2], 0, C, 0)