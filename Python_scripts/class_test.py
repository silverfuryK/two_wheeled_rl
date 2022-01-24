import pybullet as p
import time
import math
from datetime import datetime
from main import env


GRAVITY = -9.8
dt = 1e-1
iters = 2000

import pybullet_data

p.connect(p.GUI)
p.setGravity(0, 0, GRAVITY)
p.setTimeStep(dt)
path = 'two_wheel_bot_urdf4/urdf/two_wheel_bot_urdf4.urdf'
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)

env = env(path,dt,GRAVITY)
x1_pathID = p.addUserDebugParameter("x1", 0.01,0.299,0.01)
x2_pathID = p.addUserDebugParameter("x2", 0.01,0.299,0.01)
y1_pathID = p.addUserDebugParameter("y1", 0.01,0.299,0.01)
y2_pathID = p.addUserDebugParameter("y2", 0.01,0.299,0.01)

while(1):
    x11 = p.readUserDebugParameter(x1_pathID)
    x22 = p.readUserDebugParameter(x2_pathID)
    y11 = p.readUserDebugParameter(y1_pathID)
    y22 = p.readUserDebugParameter(y2_pathID)

    a = env.get_leg_length1()
    #print(a)
    env.set_leg_length1(x11,x22,y11,y22)
    env.step_simulation()
    a = env.get_leg_length1()
    #print(a)
    #print(1/10)
    time.sleep(1/240)