# Import libraries
from fr_utils import *
import numpy as np
from copy import copy

# ==========================
# Constants for the robot
# ==========================

# Obtain from DH parameters
A = 0.077476 # Thigh offset * -1 
B = 0.2115 # Thigh to Scalp offset
C = 0.2309 # Scalp to toe offset

# Transform from base to toe
leg_offset_x = 0.196
leg_offset_y = 0.049664

# =================================
# Forward Kinematics for each leg
# ================================

# Front Right Leg from base
def fk_FR(q,type = 'tran',joint = 3,group=False):
    """
    Returns the Transformation Matrix from the robot base to the Front Right Joints
    According to the 'joint' variale, the joint is selected. If the 'group' variable is True 
    (false by default) it returns the accumulated array of transformation matrix
    """
    
    # Obtain from DH parameters
    TFR_01 = dh(q[0] + np.pi, 0, 0, np.pi/2)
    TFR_12 = dh(q[1] + np.pi, -A, B, 0)
    TFR_23 = dh(q[2], 0, C, 0)

    TFR_B0 = sTrasl(leg_offset_x, -leg_offset_y, 0).dot(sTroty(np.pi/2))   
    TFR_B1 = TFR_B0.dot(TFR_01)
    TFR_B2 = TFR_B0.dot(TFR_01).dot(TFR_12)
    TFR_B3 = TFR_B0.dot(TFR_01).dot(TFR_12).dot(TFR_23)

    T = [TFR_B0,TFR_B1,TFR_B2,TFR_B3]

    if group and type == 'tran':
        return T[0:joint+1]
    else:
        if type == 'pos': 
            T_r = T[joint]
            return T_r[0:3,3]
        elif type == 'tran':
            return T[joint]

    
# Front Left Leg from base
def fk_FL(q,type = 'tran',joint = 3,group=False):
    """
    Returns the Transformation Matrix from the robot base to the Front Left Joints
    According to the 'joint' variale, the joint is selected. If the 'group' variable is True 
    (false by default) it returns the accumulated array of transformation matrix
    """

    TFL_01 = dh(q[0] + np.pi, 0, 0, np.pi/2)
    TFL_12 = dh(q[1] + np.pi, A, B, 0)
    TFL_23 = dh(q[2], 0, C, 0)

    TFL_B0 = sTrasl(leg_offset_x, leg_offset_y, 0).dot(sTroty(np.pi/2))
    TFL_B1 = TFL_B0.dot(TFL_01)
    TFL_B2 = TFL_B0.dot(TFL_01).dot(TFL_12)
    TFL_B3 = TFL_B0.dot(TFL_01).dot(TFL_12).dot(TFL_23)
    
    T = [TFL_B0,TFL_B1,TFL_B2,TFL_B3]

    if group and type=='tran':
        return T[0:joint+1]
    else:        
        if type == 'pos': 
            T_r = T[joint]
            return T_r[0:3,3]
        elif type == 'tran':
            return T[joint]


# Rear Right Leg from base
def fk_RR(q,type = 'tran',joint = 3,group=False):
    """
    Returns the Transformation Matrix from the robot base to the Rear Right Joints
    According to the 'joint' variale, the joint is selected. If the 'group' variable is True 
    (false by default) it returns the accumulated array of transformation matrix
    """

    TRR_01 = dh(q[0] + np.pi, 0, 0, np.pi/2)
    TRR_12 = dh(q[1] + np.pi, -A, B, 0)
    TRR_23 = dh(q[2], 0, C, 0)

    TRR_B0 = sTrasl(-leg_offset_x,-leg_offset_y, 0).dot(sTroty(np.pi/2))
    TRR_B1 = TRR_B0.dot(TRR_01)
    TRR_B2 = TRR_B0.dot(TRR_01).dot(TRR_12)
    TRR_B3 = TRR_B0.dot(TRR_01).dot(TRR_12).dot(TRR_23)
 
    T = [TRR_B0,TRR_B1,TRR_B2,TRR_B3]

    if group and type=='tran':
        return T[0:joint+1]
    else: 
        if type == 'pos':
            T_r = T[joint]
            return T_r[0:3,3]
        elif type == 'tran':
            return T[joint]


# Rear Left Leg from base
def fk_RL(q,type = 'tran',joint = 3, group=False):
    """
    Returns the Transformation Matrix from the robot base to the Rear Left Joints
    According to the 'joint' variale, the joint is selected. If the 'group' variable is True 
    (false by default) it returns the accumulated array of transformation matrix
    """

    TRL_01 = dh(q[0] + np.pi, 0, 0, np.pi/2)
    TRL_12 = dh(q[1] + np.pi, A, B, 0)
    TRL_23 = dh(q[2], 0, C, 0)

    TRL_B0 = sTrasl(-leg_offset_x, leg_offset_y, 0).dot(sTroty(np.pi/2))
    TRL_B1 = TRL_B0.dot(TRL_01)
    TRL_B2 = TRL_B0.dot(TRL_01).dot(TRL_12)
    TRL_B3 = TRL_B0.dot(TRL_01).dot(TRL_12).dot(TRL_23)

    T = [TRL_B0,TRL_B1,TRL_B2,TRL_B3]

    if group and type == 'tran': 
        return T[0:joint+1]
    else:
        if type == 'pos': 
            T_r = T[joint]
            return T_r[0:3,3]
        elif type == 'tran':
            return T[joint]


def fk_robot(q,leg,type='tran',joint=3,group=False): 
    if leg == 'FR':
        return fk_FR(q,type,joint,group)
    elif leg == 'FL':
        return fk_FL(q,type,joint,group)
    elif leg == 'RR': 
        return fk_RR(q,type,joint,group)
    elif leg == 'RL':
        return fk_RL(q,type,joint,group)
    

# =================================
# Jacobian calculations
# ================================

def Jgeom_leg(q,leg,joint=3):
    """
    Function that calculates the Geometric Jacobian for Linear and 
    Angular velocities of the robot from the base to the desired joint 
    """
    T_ref = fk_robot(q,leg,'tran',joint,True)
    n_t = len(T_ref)

    J = np.zeros((6,n_t-1))

    for n in range(1,n_t):
        z_0n = T_ref[n-1][0:3,2]
        p_0n = T_ref[n-1][0:3,3]
        Jv = crossproduct(z_0n, T_ref[n_t-1][0:3,3]- p_0n)
        Jw = z_0n        
        J[:,n-1] = np.hstack((Jv,Jw))

    return J



def Jan_leg(q,leg,joint=3, delta = 0.001): 
    """
    Function that calculates the Analytical Jacobian for the Cartesian and 
    Quaternion representation based on the desired joint of one leg
    """

    JT = np.zeros((joint,7))
    T = fk_robot(q,leg,'tran',joint,False)

    Q = rot2quaternion(T[0:3,0:3])
    p = T[0:3,3]
    pose = np.hstack((p,Q))

    for i in range(3):
        dq = copy(q)
        dq[i] = dq[i] + delta
        dT = fk_robot(dq,leg,'tran',joint,False)
        dQ = rot2quaternion(dT[0:3,0:3])
        dp = dT[0:3,3]
        dpose = np.hstack((dp,dQ))

        JT [i,:] = (dpose - pose)/delta

    return JT.T

