import pybullet as p
import time
import math
from datetime import datetime

class env:
    def __init__(self,urdf_path, time_step, gravity):

        self.botID = p.loadURDF(urdf_path, [0, 0, 0.4], useFixedBase= True)
        #self.botID = botID
        self.time_step = time_step
        self.gravity = gravity
        self.link_len = 0.15
        self.l1 = 0
        self.l2 = 0

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
        alpha1 = -(p.getJointState(self.botID,0)[0])
        #print(alpha1)
        beta1 = -(p.getJointState(self.botID,1)[0] + abs(alpha1))
        #print(beta1)
        alpha2 = (p.getJointState(self.botID,3)[0])
        #print(alpha2)
        beta2 = (p.getJointState(self.botID,4)[0] - alpha2)
        #print(beta2)
        self.link_len = 0.15
        l1_1 = self.link_len*math.cos(-(alpha1))
        l1_2 = self.link_len*math.cos(-(beta1))
        l2_1 = self.link_len*math.cos((alpha2))
        l2_2 = self.link_len*math.cos((beta2))
        self.l1 = l1_1 + l1_2
        self.l2 = l2_1 + l2_2
        return self.l1, self.l2

    def get_leg_length1(self):
        alpha1 = -(p.getJointState(self.botID,0)[0])
        #print(alpha1)
        beta1 = -(p.getJointState(self.botID,1)[0])
        #print(beta1)
        alpha2 = (p.getJointState(self.botID,3)[0])
        #print(alpha2)
        beta2 = (p.getJointState(self.botID,4)[0])
        #print(beta2)

        x1 = self.link_len*math.cos(alpha1) + self.link_len*math.cos(beta1)
        y1 = self.link_len*math.sin(alpha1) + self.link_len*math.sin(beta1)

        x2 = self.link_len*math.cos(alpha2) + self.link_len*math.cos(beta2)
        y2 = self.link_len*math.sin(alpha2) + self.link_len*math.sin(beta2)

        return [[x1,y1],[x2,y2]]

    def set_leg_length(self,l1,l2):
        l1_1 = l1/2
        l1_2 = l1/2
        l2_1 = l2/2
        l2_2 = l2/2

        alpha1 = math.acos(l1_1/self.link_len)
        beta1 = 2*alpha1
        alpha2 = math.acos(l2_1/self.link_len)
        beta2 = 2*alpha2

        p.setJointMotorControl2(self.botID, 0, p.POSITION_CONTROL, targetPosition = -alpha1)
        p.setJointMotorControl2(self.botID, 1, p.POSITION_CONTROL, targetPosition = -beta1)

        p.setJointMotorControl2(self.botID, 3, p.POSITION_CONTROL, targetPosition = alpha2)
        p.setJointMotorControl2(self.botID, 4, p.POSITION_CONTROL, targetPosition = beta2)

        return
    
    def set_leg_length1(self,l1,l2):
        #no lateral movement
        l1 = -l1
        l2 = -l2
        b1 = math.acos((l1**2 - 2*self.link_len**2)/(2*self.link_len**2))
        a1 = math.pi/2 - math.atan((self.link_len*math.sin(b1))/(self.link_len+self.link_len*math.cos(b1)))
        b2 = math.acos((l2**2 - 2*self.link_len**2)/(2*self.link_len**2))
        a2 = math.pi/2 - math.atan((self.link_len*math.sin(b2))/(self.link_len+self.link_len*math.cos(b2)))

        p.setJointMotorControl2(self.botID, 0, p.POSITION_CONTROL, targetPosition = -a1)
        p.setJointMotorControl2(self.botID, 1, p.POSITION_CONTROL, targetPosition = -b1)

        p.setJointMotorControl2(self.botID, 3, p.POSITION_CONTROL, targetPosition = a2)
        p.setJointMotorControl2(self.botID, 4, p.POSITION_CONTROL, targetPosition = b2)


        return


    def step_simulation(self):
        p.stepSimulation()
    
    def observations(self):
        pose = p.getBasePositionAndOrientation(self.botID)[0]
        rpy = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.botID)[1])



