import pybullet as p
import time
import pybullet_data
import os


# Initiate Pybullet interface
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.setAdditionalSearchPath(r'C:\Users\raul_\Documents\Programming\Robótica Avanzada\Proyecto\RobAv_Proyect_CorEsc\Body')

# Load plane and variables
plane = p.loadURDF("plane.urdf")
p.setGravity(0,0,0)
p.setTimeStep(1./500)


# Load Quadruped and Settings - Enable collision between lower legs
urdfFlags = p.URDF_USE_SELF_COLLISION
quadruped = p.loadURDF("Laikago/laikago_toes.urdf",[0,0,1.5],[0,0.5,0.5,0], flags = urdfFlags,useFixedBase=False)

print(p.getBodyInfo(quadruped))