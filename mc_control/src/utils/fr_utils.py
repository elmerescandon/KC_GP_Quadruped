import numpy as np



cos = np.cos
sin = np.sin

def dh(th, d, a, alpha):

    cth = np.cos(th); sth = np.sin(th)
    ca = np.cos(alpha); sa = np.sin(alpha)
    Tdh = np.array([ [cth, -ca*sth,  sa*sth, a*cth],
                     [sth,  ca*cth, -sa*cth, a*sth],
                     [0,        sa,     ca,      d],
                     [0,         0,      0,      1]])
    return Tdh


# def rot2quaternion(R):

#     omega = ((1+R[0, 0]+R[1, 1]+R[2, 2])**0.5)*0.5
#     if omega == 0: 
#         quat = Quaternion(matrix=R)
#         q = np.array([q[0],q[1],q[2],q[3]])
#         return q
#     else :
#         ex = (1/(4*omega))*(R[2, 1]-R[1, 2])
#         ey = (1/(4*omega))*(R[0, 2]-R[2, 0])
#         ez = (1/(4*omega))*(R[1, 0]-R[0, 1])
#         return np.array([omega,ex,ey,ez])


def rot(th, a):
    th = np.deg2rad(th)
    if (a == 'x'):
        R = np.array([[1,       0,        0],
                      [0, cos(th), -sin(th)],
                      [0, sin(th),  cos(th)]])
    elif (a == 'y'):
        R = np.array([[cos(th),      0, sin(th)],
                      [0,      1,      0],
                      [-sin(th),     0, cos(th)]])
    elif (a == 'z'):
        R = np.array([[cos(th), -sin(th), 0],
                      [sin(th), cos(th), 0],
                      [0,      0,    1]])
    return R

def sTrasl(x, y, z):

    T = np.array([[1,0,0,x],
                    [0,1,0,y],
                    [0,0,1,z],
                    [0,0,0,1]])
    return T

def sTrotx(ang):
    Tx = np.array([[1, 0,0,0],
                    [0, np.cos(ang),-np.sin(ang),0],
                    [0, np.sin(ang), np.cos(ang),0],
                    [0, 0, 0, 1]])
    return Tx

def sTroty(ang):
    Ty = np.array([[np.cos(ang),0,np.sin(ang),0],
                    [0,1,0,0],
                    [-np.sin(ang),0,np.cos(ang),0],
                    [0,0,0,1]])
    return Ty

def sTrotz(ang):
    Tz = np.array([[np.cos(ang),-np.sin(ang),0,0],
                    [np.sin(ang), np.cos(ang),0,0],
                    [0,0,1,0],
                    [0,0,0,1]])
    return Tz

def crossproduct(a, b):
    x = np.array([[a[1]*b[2] - a[2]*b[1]],
                  [a[2]*b[0] - a[0]*b[2]],
                  [a[0]*b[1] - a[1]*b[0]]])
    return x

def quaternionMult(q1, q2):

    qout = np.zeros(4)
    qout[0] = -q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3] + q1[0] * q2[0]
    qout[1] = q1[0] * q2[1] - q1[3] * q2[2] + q1[2] * q2[3] + q1[1] * q2[0]
    qout[2] = q1[3] * q2[1] + q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0]
    qout[3] = -q1[2] * q2[1] + q1[1] * q2[2] + q1[0] * q2[3] + q1[3] * q2[0]
    return qout