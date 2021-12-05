import numpy as np



cos = np.cos
sin = np.sin




def dh(th, d, a, alpha):

    """"
    Obtain transform matri from the Denavit-Hatenberg parameters
    """
    cth = np.cos(th); sth = np.sin(th)
    ca = np.cos(alpha); sa = np.sin(alpha)
    Tdh = np.array([ [cth, -ca*sth,  sa*sth, a*cth],
                     [sth,  ca*cth, -sa*cth, a*sth],
                     [0,        sa,     ca,      d],
                     [0,         0,      0,      1]])
    return Tdh



def rot(th, a):
    """
    Generate the Rotation matrix from an angle 'th' in 
    degrees around an 'a' axis (input as a character variable)
    """
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
    """
    Generate the Transform matrix only for 
    traslation in the three Cartesian axis
    """

    T = np.array([[1,0,0,x],
                    [0,1,0,y],
                    [0,0,1,z],
                    [0,0,0,1]])
    return T

def sTrotx(ang):
    """
    Generate the Transform matrix only for 
    a rotation in the X axis
    """

    Tx = np.array([[1, 0,0,0],
                    [0, np.cos(ang),-np.sin(ang),0],
                    [0, np.sin(ang), np.cos(ang),0],
                    [0, 0, 0, 1]])
    return Tx

def sTroty(ang):
    """
    Generate the Transform matrix only for 
    a rotation in the Y axis
    """
    Ty = np.array([[np.cos(ang),0,np.sin(ang),0],
                    [0,1,0,0],
                    [-np.sin(ang),0,np.cos(ang),0],
                    [0,0,0,1]])
    return Ty

def sTrotz(ang):
    """
    Generate the Transform matrix only for 
    a rotation in the Z axis
    """
    Tz = np.array([[np.cos(ang),-np.sin(ang),0,0],
                    [np.sin(ang), np.cos(ang),0,0],
                    [0,0,1,0],
                    [0,0,0,1]])
    return Tz

def crossproduct(a, b):
    """
    Returns the cross product between two vectors
    """
    x = np.array([a[1]*b[2] - a[2]*b[1],
                  a[2]*b[0] - a[0]*b[2],
                  a[0]*b[1] - a[1]*b[0]])    
    
    return x

def quaternionMult(q1, q2):
    """
    Returns the product between two Quaternion representation
    """

    qout = np.zeros(4)
    qout[0] = -q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3] + q1[0]*q2[0]
    qout[1] =  q1[0]*q2[1] - q1[3]*q2[2] + q1[2]*q2[3] + q1[1]*q2[0]
    qout[2] =  q1[3]*q2[1] + q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0]
    qout[3] = -q1[2]*q2[1] + q1[1]*q2[2] + q1[0]*q2[3] + q1[3]*q2[0]
    return qout

def rot2quaternion(R):
    """
    Function that returns the quaternion unit from a Rotation Matrix
    Output shown as:
    q = (ex,ey,ez,w)
    Obtained from:  https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    """
    trace = R[0,0] + R[1,1] + R[2,2]

    if trace>0:
        s = 0.5/np.sqrt(trace + 1.0)
        w = 0.25/s
        x = ( R[2,1] - R[1,2] )*s 
        y = ( R[0,2] - R[2,0] )*s
        z = ( R[1,0] - R[0,1] )*s
    else: 
        if (R[0,0] > R[1,1] ) and (R[0,0] > R[2,2]):
            s = 2.0*np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2] )
            w = (R[2,1] - R[1,2])/s
            x = 0.25*s
            y = (R[0,1] + R[1,0])/s 
            z = (R[0,2] + R[2,0])/s
        
        elif R[1,1] > R[2,2]:
            s = 2.0*np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
            w = (R[0,2] - R[2,0])/s
            x = (R[0,1] + R[1,0])/s
            y = 0.25*s
            z = (R[1,2] + R[2,1])/s
        else: 
            s = 2.0*np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
            w = (R[1,0] - R[0,1])/s
            x = (R[0,2] + R[2,0])/s
            y = (R[1,2] + R[2,1])/s
            z = 0.25*s
    
    return np.array([w,x,y,z])

def TQb(quat): 
    w = quat[0]
    x = quat[1]
    y = quat[2]
    z = quat[3]

    T = 2*np.array([[-x,w,-z,y],
                    [-y,z,w,-x],
                    [-z,-y,x,w]])
    return T

def quaternion2rot(Q):
    '''
    Function that returns the Rotation Matrix from a 
    Quaternion. Check:
    http://oramosp.epizy.com/teaching/212/fund-robotica/clases/2_Representaciones_Espaciales_II.pdf
    '''
    w = Q[0]
    ex = Q[1]
    ey = Q[2]
    ez = Q[3]

    R = np.array([[2*(w**2 + ex**2)-1 ,2*(ex*ey - w*ez)   ,2*(ex*ez + w*ey)], 
                  [2*(ex*ey +  w*ez)  ,2*(w**2 + ey**2)-1 ,2*(ey*ez - w*ex),], 
                  [2*(ex*ez - w*ey)   ,2*(ey*ez + w*ex)   ,2*(w**2 + ez**2)-1]])
    
    return R

def skew_matrix(p):

    ux = p[0]
    uy = p[1]
    uz = p[2]
    return np.array([[0, -uz, uy],
                     [uz, 0, -ux],
                     [-uy, ux, 0]])
    
def diffQuat(Q,Qd):
    w = Q[0]
    x = Q[1]
    y = Q[2]
    z = Q[3]

    wd = Qd[0]
    xd = Qd[1]
    yd = Qd[2]
    zd = Qd[3]

    we = wd*w + (np.resize(Qd[1:4],(3,1)).T).dot(Q[1:4])
    ee = -wd*Q[1:4] + w*Qd[1:4] - crossproduct(Qd[1:4],Q[1:4])

    print(we.shape)
    print(ee.shape)

    return np.concatenate((we,ee))