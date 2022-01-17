import pybullet as p
import time
import math
from datetime import datetime

class env:
    def __init__(self,botID, time_step, gravity):
        self.botID = botID
        self.time_step = time_step
        self.gravity = gravity

        """
        JOINT INFO

        joint0 = joint1L
        joint1 = joint2L
        joint2 = wheel_jointL
        joint3 = joint1R
        joint4 = joint2R
        joint5 = wheel_jointR
        
        """
        
        p.setGravity(0, 0, self.gravity)

    def get_leg_length(self):
        l1 = 0
        l2 = 0
        alpha1 = p.getJointState(self.botID,0)
        beta1 = p.getJointState(self.botID,1)
        alpha2 = p.getJointState(self.botID,2)
        beta2 = p.getJointState(self.botID,3)

        
        return l1,l2

    def step_simulation(self):
        p.stepSimulation()
    
    def observations(self):
        pose = p.getBasePositionAndOrientation(self.botID)[0]
        rpy = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.botID)[1])



