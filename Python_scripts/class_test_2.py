import pybullet as p
import time
import math
from datetime import datetime
from main import env
from traj import Trajectory
from a_c_test_agent import NewAgent
from utils import plotLearning
import numpy as np
from ddpg_ac import Agent

GRAVITY = -9.8
dt = 1e-1
iters = 2000
simu_time = 0.0
pi = math.pi

import pybullet_data

p.connect(p.GUI)
#p.connect(p.DIRECT)
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

#agent = Agent(alpha=0.000005, beta=0.00001, input_dims=[24], gamma=0.99,
#                  layer1_size=256, layer2_size=256, n_actions=6)

#ddpg

agent = Agent(alpha=0.0025, beta=0.0025, input_dims=[24], tau=0.001, env=env,
              batch_size=64,  layer1_size=512, layer2_size=512, n_actions=6)
agent.load_models()
#agent.check_actor_params()
'''
try: 
        agent.load_models()
        agent.check_actor_params()
except:
        print("no saved models found!!")
'''
score_history = []
i = 0
tot_episodes = 1
max_tp = 60*60*10*10*2
tp = 0
for i in range(tot_episodes):
        obs = env.reset()
        done = False
        score = 0
        while tp != max_tp:
                '''
                cmd_vel = trajec.get_cmd_vel(sim_time)
                #print(cmd_vel)
                #env.action([0,0,0.1,0,0,0.1])
                action = np.array(agent.choose_action(observation)).reshape((1,))
                print(action)
                #p.stepSimulation()
                observation_, reward, done = env.step_simulation(action)
                agent.learn(observation, reward, observation_, done)
                observation = observation_
                #print(done)
                #print([env.obs_t[2],env.obs_t[3],env.obs_t[6],env.obs_t[7],env.obs_t[11]])
                score += reward
                #time.sleep(dt)
                '''
                act = agent.choose_action(obs)
                new_state, reward, done = env.step_simulation(act)
                #print(new_state)
                agent.remember(obs, act, reward, new_state, int(done))
                agent.learn()
                score += reward
                obs = new_state
                #time.sleep(dt)
                #env.render()
                
                print('timestep: ', tp,'sim time: %.2f'% env.sim_time,' reward: ',env.reward_t)
                tp = tp + 1
        score_history.append(score)
        print('episode: ', i,'score: %.2f' % score,'sim time: %.2f'% env.sim_time,' reward: ',env.reward_t)
        #print('sim time: %.2f'% env.sim_time,' reward: ',env.reward_t)
        #print(env.obs_t, env.action)
        #print(env.reward_t)
print(agent.actor.checkpoint_file)
agent.save_models()      
plotLearning(score_history, filename = 'plot1.png', window=20)
print("DONE lol")