import pybullet as p
import time
import pybullet_data

# Code
# Pybulletw works with a server-client framework
# 
physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical 
p.setAdditionalSearchPath(pybullet_data.getDataPath()) # optionally
p.setGravity(0,0,-9.81)

# Generate form in the simulation
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("samurai.urdf",cubeStartPos,cubeStartOrientation)



# Generate simulation 
for i in range(10000): 
    p.stepSimulation()
    time.sleep(1./240.)

cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()

