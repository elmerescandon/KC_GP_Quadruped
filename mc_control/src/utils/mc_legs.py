# Import libraries
from fr_utils import *
import numpy as np

# Front Right Leg from base
def fk_FR(q,type = 'tran'):
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]

    # Obtain from DH parameters
    A = 0.077476 # Thigh offset * -1 
    B = 0.2115
    C = 0.2309

    T01 = dh(q1 + np.pi, 0, 0, np.pi/2)
    T12 = dh(q2 + np.pi, -A, B, 0)
    T23 = dh(q3, 0, C, 0)
    T03 = T01.dot(T12).dot(T23)

    # Transform from base to toe
    leg_offset_x = 0.196
    leg_offset_y = -0.049664
    T_B = sTrasl(leg_offset_x, leg_offset_y, 0)
    T_R = sTroty(np.pi/2)
    TBR_0 = T_B.dot(T_R).dot(T03)
    

    if type == 'pos': 
        return TBR_0[0:3,3]
    elif type == 'tran':
        return TBR_0

    
# Front Left Leg from base
def fk_FL(q,type = 'tran'):
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]

    # Obtain from DH parameters
    A = 0.077476 # Thigh offset * -1 
    B = 0.2115
    C = 0.2309

    T01 = dh(q1 + np.pi, 0, 0, np.pi/2)
    T12 = dh(q2 + np.pi, A, B, 0)
    T23 = dh(q3, 0, C, 0)
    T03 = T01.dot(T12).dot(T23)

    # Transform from base to toe
    leg_offset_x = 0.196
    leg_offset_y = 0.049664
    T_B = sTrasl(leg_offset_x, leg_offset_y, 0)
    T_R = sTroty(np.pi/2)
    TBR_0 = T_B.dot(T_R).dot(T03)
    

    if type == 'pos': 
        return TBR_0[0:3,3]
    elif type == 'tran':
        return TBR_0


# Rear Right Leg from base
def fk_RR(q,type = 'tran'):
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]

    # Obtain from DH parameters
    A = 0.077476 # Thigh offset * -1 
    B = 0.2115
    C = 0.2309

    T01 = dh(q1 + np.pi, 0, 0, np.pi/2)
    T12 = dh(q2 + np.pi, -A, B, 0)
    T23 = dh(q3, 0, C, 0)
    T03 = T01.dot(T12).dot(T23)

    # Transform from base to toe
    leg_offset_x = -0.196
    leg_offset_y = -0.049664
    T_B = sTrasl(leg_offset_x, leg_offset_y, 0)
    T_R = sTroty(np.pi/2)
    TBR_0 = T_B.dot(T_R).dot(T03)
    

    if type == 'pos': 
        return TBR_0[0:3,3]
    elif type == 'tran':
        return TBR_0


# Rear Left Leg from base
def fk_RL(q,type = 'tran'):
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]

    # Obtain from DH parameters
    A = 0.077476 # Thigh offset * -1 
    B = 0.2115
    C = 0.2309

    T01 = dh(q1 + np.pi, 0, 0, np.pi/2)
    T12 = dh(q2 + np.pi, A, B, 0)
    T23 = dh(q3, 0, C, 0)
    T03 = T01.dot(T12).dot(T23)

    # Transform from base to toe
    leg_offset_x = -0.196
    leg_offset_y = 0.049664
    T_B = sTrasl(leg_offset_x, leg_offset_y, 0)
    T_R = sTroty(np.pi/2)
    TBR_0 = T_B.dot(T_R).dot(T03)
    

    if type == 'pos': 
        return TBR_0[0:3,3]
    elif type == 'tran':
        return TBR_0
