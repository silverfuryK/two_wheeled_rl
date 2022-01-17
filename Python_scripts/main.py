import pybullet as p
import time
import math
from datetime import datetime

class env:
    def __init__(self,botID, time_step, gravity):
        self.botID = botID
        self.time_step = time_step
        self.gravity = gravity
        
        p.setGravity(0, 0, self.gravity)

    def step_simulation(self):
        p.stepSimulation()
    
    def observations(self):
        pose = p.getBasePositionAndOrientation(self.botID)[0]
        rpy = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.botID)[1])
        

