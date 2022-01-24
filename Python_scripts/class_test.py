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

    l1,l2 = env.get_leg_length1()
    print([l1,l2])
    env.set_leg_length1(0.15,0.15)
    env.step_simulation()
    l1,l2 = env.get_leg_length1()
    print([l1,l2])
    time.sleep(1/10)