import pybullet as p
import time
import math
from datetime import datetime
from main import env
from traj import Trajectory

GRAVITY = -9.8
dt = 1e-1
iters = 2000
simu_time = 0.0
pi = math.pi

import pybullet_data

p.connect(p.GUI)
p.setGravity(0, 0, GRAVITY)
p.setTimeStep(dt)
path = 'two_wheel_bot_urdf4/urdf/two_wheel_bot_urdf4.urdf'
p.setAdditionalSearchPath(pybullet_data.getDataPath())
#p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)


# trajectory

file = [[ 0.0, 0.0 , 0.0],
        [ 5.0, 1.0 , 0.0],
        [10.0, 0.0 , 0.5],
        [12.0, 2.0 , 0.0],
        [17.0, 0.0 , 0.0],
        [20.0, 0.0 , 0.0]]

num_commands = len(file)
trajec = Trajectory(file)
timestep = 0.00
sim_time = 0.00
total_timestep = 20/dt
cmd_ptr = 0

#env
env = env(path,dt,GRAVITY,file,20.0)
env.reset()


while timestep != total_timestep:

    cmd_vel = trajec.get_cmd_vel(sim_time)
    #print(cmd_vel)
    env.action([0,0,0.1,0,0,0.1])
    #p.stepSimulation()
    observations, reward, done = env.step_simulation()
    #print(env.obs_t[2])
    time.sleep(dt)