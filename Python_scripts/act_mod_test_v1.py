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
path = 'two_wheel_bot_urdf4/urdf/two_wheel_bot_urdf4.urdf'
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)
botId = p.loadURDF(path, [0, 0, 0.4], useFixedBase= True)

p.resetBasePositionAndOrientation(botId, [0, 0, 0.4], [0, 0, 0, 1])
kukaEndEffectorIndex = 6
numJoints = p.getNumJoints(botId)
print("joint = "+ str(numJoints))
print("body = "+ str(p.getNumBodies()))
#params = []
for n in range(0,6):
  print(p.getJointInfo(botId,n))

print(".   .")
for n in range(0,6):
  print(p.getLinkState(botId,n)[5])
print(".   .")
print(p.getBasePositionAndOrientation(botId))
#l = [p.getBodyInfo(0),p.getBodyInfo(1)]
l = p.getBodyInfo(botId)
print(l)


#print(params)

jointFrictionForce = 0.01
#for joint in range(p.getNumJoints(botId)):
  #p.setJointMotorControl2(botId, joint, p.POSITION_CONTROL, force=jointFrictionForce)

#import time
p.setRealTimeSimulation(1)
time.sleep(1/240)
#time.sleep(2)
#while (1):
  #time.sleep(2000)
p.stepSimulation()
for i in range(6):
  target = math.pi/4
  if i<3:
    target = -target
  
  print(target)
  
  print(i)
  print((p.getJointState(botId,i)[0]))
  p.stepSimulation()
  p.setJointMotorControl2(botId, i, p.POSITION_CONTROL, targetPosition = target)
  p.stepSimulation()
  print((p.getJointState(botId,i)[0]))
  time.sleep(1/240)
  #time.sleep(1)
#print(p.getEulerFromQuaternion(p.getBasePositionAndOrientation(botId)[1]))
#c = p.getContactPoints(0)
#print(len(c))
p.setGravity(0, 0, GRAVITY)
time.sleep(1 / 240.)
#time.sleep(1000)
while (1):
  p.stepSimulation()
  time.sleep(1/240)
  #time.sleep(1)