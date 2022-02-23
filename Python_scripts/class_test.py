import pybullet as p
import time
import math
from datetime import datetime
from main import env
from traj import Trajectory
from a_c_test_agent import NewAgent
from utils import plotLearning
import numpy as np

GRAVITY = -9.8
dt = 1e-1
iters = 2000
simu_time = 0.0
pi = math.pi

import pybullet_data

#p.connect(p.GUI)
p.connect(p.DIRECT)
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

agent = NewAgent(alpha=0.000005, beta=0.00001, input_dims=[24], gamma=0.99,
                  layer1_size=256, layer2_size=256)

score_history = []
i = 0
tot_episodes = 30
for i in range(tot_episodes):
        observation = env.reset()
        done = False
        score = 0
        while not done:
                cmd_vel = trajec.get_cmd_vel(sim_time)
                #print(cmd_vel)
                #env.action([0,0,0.1,0,0,0.1])
                action = np.array(agent.choose_action(observation)).reshape((1,))
                #p.stepSimulation()
                observation_, reward, done = env.step_simulation()
                agent.learn(observation, reward, observation_, done)
                observation = observation_
                #print(done)
                #print([env.obs_t[2],env.obs_t[3],env.obs_t[6],env.obs_t[7],env.obs_t[11]])
                score += reward
                time.sleep(dt)
        score_history.append(score)
        print('episode: ', i,'score: %.2f' % score)
        print('sim time: ',env.sim_time,' reward: ',env.reward_t)
        print(env.obs_t[2])
        #print(env.reward_t)
        
plotLearning(score_history, filename = 'plot1.png', window=20)