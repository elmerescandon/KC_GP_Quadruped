# Import libraries
from fr_utils import *
import numpy as np
from copy import copy
from mc_legs import *


class mini_cheetah: 
    def __init__(self):
        '''
            x_b: Position and Orientation (Q) of the floating-base 
            q: Array of 12 joints in the following order: RL,FL,RR,FR  [Hip,Thigh, Calf]
        '''
        # Position

        self.q = np.array([ 0,0,0, # Base: x,y,z [0:3]
                            1,0,0,0, # Quaternion base: w,x,y,z [3:7]
                            0,0,0, # RL: q1,q2,q3 [7:10]
                            0,0,0, # FL: q1,q2,q3 [10:13]
                            0,0,0, # RR: q1,q2,q3 [13:16]
                            0,0,0]) # FR: q1,q2,q3 [16:19]


        # Velocities
        self.q_dot = np.zeros((7 + 12)) # Base: 3 Position + 4 Orientation, Legs: 12 Joints

        # Miscellanous
        self.joint_dic = {'RL':[7,10],
                          'FL':[10,13], 
                          'RR':[13,16], 
                          'FR':[16,19]}

    def step_update(self,q): 
        self.q = q

    def base_position(self):
        return self.q[0:3] 
    
    def base_pose(self):
        return self.q[0:7] 

    # ==========================================================
    # Operations to obtain the pose and velocities of the robot
    # ==========================================================
    # Inspired on functions created by Oscar Ramos Floating base example


    def leg_position(self,leg,type='tran'): 
        '''
        Function that returns the position of the toe from the interial
        frame of the robot considering the floating base
        '''
        if leg == 'RL':
            q_leg = self.q[7:10]
        elif leg == 'FL':
            q_leg = self.q[10:13]
        elif leg == 'RR':
            q_leg = self.q[13:16]
        elif leg == 'FR':
            q_leg = self.q[16:19]         

        # Cartesian position of the leg
        p_b = self.q[0:3] #(x,y,z)
        Q_b = self.q[3:7]
        R_IB = quaternion2rot(Q_b)
       
        
        # Position of the leg from the base frame
        T_Bi = fk_robot(q_leg,leg)
        p_Bi = T_Bi[0:3,3]
        R_Bi = T_Bi[0:3,0:3]
        Q_Bi = rot2quaternion(R_Bi)

        # Calculate position of the leg from the inertial frame
        p_Ii = R_IB.dot(p_Bi) + p_b
        R_Ii = R_IB.dot(R_Bi)

        # Calculate Orientation 
        Q_Ii =  quaternionMult(Q_b,Q_Bi)
        x_leg = np.concatenate((p_Ii,Q_Ii))

        # Transform Matrix
        T_Ii = np.zeros((4,4))
        T_Ii[0:3,0:3]= R_Ii
        T_Ii[0:3,3] = p_Ii
        T_Ii[3,3] = 1


        if type=='pose':
            return x_leg
        elif type=='pos':
            return p_Ii
        elif type == 'tran':
            return T_Ii
    

    def leg_jacobian(self,leg):
        indx = self.joint_dic[leg]
        if leg=='RL':
            q_leg = self.q[7:10]
        elif leg=='FL':
            q_leg = self.q[10:13]
        elif leg=='RR':
            q_leg = self.q[13:16]
        elif leg=='FR':
            q_leg = self.q[16:19] 
        
        # Twist of the Leg using the Inertial frame
        J = np.zeros((6,19)) 

        # Velocity of the floating base
        p_b = self.q[0:3]
        T_Ii = self.leg_position(leg)
        p_Ii = T_Ii[0:3,3]
        Q_b = self.q[3:7]
        TQ = TQb(Q_b)
        p_skew = skew_matrix(p_b - p_Ii)

        # Rotation matrix from base to Inertial frame
        R_IB = quaternion2rot(Q_b)

        # Calculate Jacobian Matrix of the leg from the base frame
        Jgleg = Jgeom_leg(q_leg,leg)

        # Provide values to Jacobian matrix
        J[0:3,0:3] = np.eye(3)
        J[0:3,3:7] = p_skew.dot(TQ)
        J[3:6,3:7] = TQ
        J[0:3,indx[0]:indx[1]] = R_IB.dot(Jgleg[0:3,:]) # Linear Velocity Twist
        J[3:6,indx[0]:indx[1]] = R_IB.dot(Jgleg[3:6,:]) # Angular Velocity Twist

        return J


    # ==========================================================
    # Tasks for the desired parts of the body
    # ==========================================================
    # Position: Cartesian (x,y,z)
    # Pose: Complete position (Cartesian) and orientation (Quaternion) - x,y,z,w,ex,ey,ez

    def error_position_leg(self,pos_d,leg):     
        # Considerate only error in the cartesian space  
        return pos_d - self.leg_position(leg,'pos')

    def error_position_base(self,pos_d):
        return pos_d - self.q[0:3]

    def error_pose_base(self,pose_d):
        # Cartesian
        pose_base = self.q[0:7]
        e_pos = pose_d[0:3] - pose_base[0:3]

        # Quaternion
        Q_base = pose_base[3:7]
        Q_d = pose_d[3:7]
        Qe = diffQuat(Q_base,Q_d)

        # Concatenate
        e_ori = np.array([Qe[0]-1,Qe[1],Qe[2],Q[3]])
        e_pose = np.concatenate((e_pos,e_ori))

        return e_pose

    def leg_jacobian_position(self,leg):
         # Contribution of the three linear velocities from all the joints (3,19)
        return self.leg_jacobian(leg)[0:3,:]
    
    def base_jacobian_position(self):
        # The contribution in the Analytical and Geeometric velocities are the same
        J = np.zeros((7,19))
        J[0:3,0:3] = np.eye(3)
        return J[0:3,:]