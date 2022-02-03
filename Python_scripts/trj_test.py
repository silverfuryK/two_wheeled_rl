from pytest import mark
from traj import Trajectory
import matplotlib.pyplot as plt
import numpy as np

file = [[ 0.0, 0.0 , 0.0],
        [ 5.0, 1.0 , 0.0],
        [10.0, 0.0 , 0.5],
        [12.0, 2.0 , 0.0],
        [17.0, 0.0 , 0.0],
        [20.0, 0.0 , 0.0]]

num_commands = len(file)
trajec = Trajectory(file)

dt = 0.1
timestep = 0.00
time = 0.00
total_timestep = 20/0.1
cmd_ptr = 0
while timestep != total_timestep:
    a = trajec.get_cmd_vel(time)
    time = time + dt
    timestep = timestep + 1
    plt.scatter(time, a[0], c='#1f77b4', marker='o')
    plt.scatter(time, a[1], c='#ff7f0e', marker='v')
    print(time)

#plt.figure()
plt.show()

