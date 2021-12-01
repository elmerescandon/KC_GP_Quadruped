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


# Show info of each joint
# 3 GDL for movements, then the toe is a fixed joint
for j in range(p.getNumJoints(quadruped)): 
    print(p.getJointInfo(quadruped,j))

# Indicate collisions
lower_legs = [2,5,8,11]
for l0 in lower_legs:
	for l1 in lower_legs:
		if (l1>l0):
			enableCollision = 1
			print("collision for pair",l0,l1, p.getJointInfo(quadruped,l0)[12],p.getJointInfo(quadruped,l1)[12], "enabled=",enableCollision)
			p.setCollisionFilterPair(quadruped, quadruped, 2,5,enableCollision)


# Initialize variables
jointIds=[]
paramIds=[]
jointName = []
jointType = []
jointOffsets=[]
jointDirections=[-1,1,1,1,1,1,-1,1,1,1,1,1]
jointAngles=[0,0,0,0,0,0,0,0,0,0,0,0]

for i in range (4):
	jointOffsets.append(0)
	jointOffsets.append(-0.7)
	jointOffsets.append(0.7)

# Create slider in the GUI to variate the force applied in each revolute joint
maxForceId = p.addUserDebugParameter("maxForce",0,100,20)
# maxForceId = 20.0

# Put joint info into arrays - and create an Slider 
index = 0
for j in range (p.getNumJoints(quadruped)):
        p.changeDynamics(quadruped,j,linearDamping=0, angularDamping=0)
        info = p.getJointInfo(quadruped,j)
        js = p.getJointState(quadruped,j)
        jointName.append(info[1])
        jointType.append(info[2])
        if (jointType[j] == p.JOINT_PRISMATIC or jointType[j]==p.JOINT_REVOLUTE):
            jointIds.append(j)
            joint_slider = p.addUserDebugParameter(jointName[j].decode("utf-8"),-4,4,(js[0]-jointOffsets[index])/jointDirections[index])
            paramIds.append(joint_slider)
            index=index+1

# GUI Param initialization
p.getCameraImage(480,320)
p.setRealTimeSimulation(0)

p.setRealTimeSimulation(1)

print("Acabo")
while (1):
	
	for i in range(len(paramIds)):
		c = paramIds[i]
		targetPos = p.readUserDebugParameter(c)
		maxForce = p.readUserDebugParameter(maxForceId)
		p.setJointMotorControl2(quadruped,jointIds[i],p.POSITION_CONTROL,jointDirections[i]*targetPos+jointOffsets[i], force=maxForce)