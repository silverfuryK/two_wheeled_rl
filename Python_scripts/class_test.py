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
while(1):

    a = env.get_leg_length1()
    print(a)
    env.set_leg_length1(0.1,0.1,0.00001,0.00001)
    env.step_simulation()
    a = env.get_leg_length1()
    print(a)
    time.sleep(1/10)