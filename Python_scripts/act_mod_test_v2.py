import pybullet as p
import time
import math
from datetime import datetime
from traj import Trajectory
from future_env import env
import random
import numpy as np

GRAVITY = -9.8
dt = 1e-3
iters = 2000

#clid = p.connect(p.SHARED_MEMORY)
import pybullet_data

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

N = 1

env1 = env(False, 'two_wheel_bot_urdf4/urdf/two_wheel_bot_urdf4.urdf',dt,
            GRAVITY, file, 20.0, N, 24, 6)

env1.reset()


while (1):
  act = np.random.uniform(-1,1,(6,1))
  new_state, reward, done, sim_t, pred_obs_buffer = env1.step_simulation(act)
  #print(pred_obs_buffer)
  #time.sleep(1)
