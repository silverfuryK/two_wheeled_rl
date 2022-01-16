import pybullet as p
import time
import math
from datetime import datetime

GRAVITY = -9.8
dt = 1e-3
iters = 2000

#clid = p.connect(p.SHARED_MEMORY)
import pybullet_data

p.connect(p.GUI)
p.setGravity(0, 0, GRAVITY)
p.setTimeStep(dt)
path = 'G:\\Robotics\\tw_wh2_1\\Python_scripts\\urdf\\'
botId = p.loadURDF(path+"two_wheel_bot_urdf2.urdf", [0, 0, -0.3], useFixedBase= False)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, -0.3], useFixedBase=True)

p.resetBasePositionAndOrientation(botId, [0, 0, 0], [0, 0, 0, 1])
kukaEndEffectorIndex = 6
numJoints = p.getNumJoints(botId)
print("joint = "+ str(numJoints))
print("body = "+ str(p.getNumBodies()))
#params = []
print(p.getJointInfo(botId,0))
print(p.getJointInfo(botId,1))
print(p.getJointInfo(botId,2))
print(p.getJointInfo(botId,3))
print(p.getJointInfo(botId,4))
print(p.getJointInfo(botId,5))
print(".   .")
print(p.getBasePositionAndOrientation(botId))
#l = [p.getBodyInfo(0),p.getBodyInfo(1)]
l = p.getBodyInfo(botId)
print(l)


#print(params)

jointFrictionForce = 0.01
#for joint in range(p.getNumJoints(botId)):
  #p.setJointMotorControl2(botId, joint, p.POSITION_CONTROL, force=jointFrictionForce)

import time
p.setRealTimeSimulation(1)
time.sleep(2)
while (1):
  #time.sleep(2000)
  p.stepSimulation()
  p.setJointMotorControl2(botId, 0, p.POSITION_CONTROL, targetPosition = 0.785)
  #print(p.getEulerFromQuaternion(p.getBasePositionAndOrientation(botId)[1]))
  c = p.getContactPoints(0)
  print(len(c))
  p.setGravity(0, 0, GRAVITY)
  time.sleep(1 / 240.)
time.sleep(1000)