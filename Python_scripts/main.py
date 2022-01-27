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
        #baselink pos
        self.x_t = 0
        self.y_t = 0
        self.z_t = 0
        self.xd_t = 0
        self.yd_t = 0
        self.zd_t = 0

        #baselink rot
        self.phi_t = 0
        self.the_t = 0
        self.shi_t = 0
        self.phid_t = 0
        self.thed_t = 0
        self.shid_t = 0

        #joint states
        self.a1_t = 0
        self.b1_t = 0
        self.a2_t = 0
        self.b2_t = 0
        self.w1_t = 0
        self.w2_t = 0

        #joint vel
        self.a1d_t = 0
        self.b1d_t = 0
        self.a2d_t = 0
        self.b2d_t = 0
        self.w1d_t = 0
        self.w2d_t = 0

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
        return [self.l1, self.l2]

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
    
    def set_leg_length1(self,x1,x2,y1,y2):
        #no lateral movement
        x1 = x1
        x2 = x2
        y1 = -y1
        y2 = -y2
        b1 = math.acos((y1**2 + x1**2 - 2*self.link_len**2)/(2*self.link_len**2))
        a1 = math.atan(y1/x1) - math.atan((self.link_len*math.sin(b1))/(self.link_len+self.link_len*math.cos(b1)))

        b2 = math.acos((y2**2 + x2**2 - 2*self.link_len**2)/(2*self.link_len**2))
        a2 = math.atan(y2/x2) - math.atan((self.link_len*math.sin(b2))/(self.link_len+self.link_len*math.cos(b2)))

        p.setJointMotorControl2(self.botID, 0, p.POSITION_CONTROL, targetPosition = -a1)
        p.setJointMotorControl2(self.botID, 1, p.POSITION_CONTROL, targetPosition = -b1)

        p.setJointMotorControl2(self.botID, 3, p.POSITION_CONTROL, targetPosition = a2)
        p.setJointMotorControl2(self.botID, 4, p.POSITION_CONTROL, targetPosition = b2)


        return


    def step_simulation(self):
        p.stepSimulation()
    
    def observations(self):

        #baselink pos
        self.x_t = p.getBasePositionAndOrientation(self.botID)[0][0]
        self.y_t = p.getBasePositionAndOrientation(self.botID)[0][1]
        self.z_t = p.getBasePositionAndOrientation(self.botID)[0][2]
        self.xd_t = p.getLinkState(self.botID,0,1)[6][0]
        self.yd_t = p.getLinkState(self.botID,0,1)[6][1]
        self.zd_t = p.getLinkState(self.botID,0,1)[6][2]

        #baselink rot
        self.phi_t = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.botID)[1])[0]
        self.the_t = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.botID)[1])[1]
        self.shi_t = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.botID)[1])[2]
        self.phid_t = p.getLinkState(self.botID,0,1)[7][0]
        self.thed_t = p.getLinkState(self.botID,0,1)[7][1]
        self.shid_t = p.getLinkState(self.botID,0,1)[7][2]

        #joint states
        self.a1_t = p.getJointState(self.botID,0)[0]
        self.b1_t = p.getJointState(self.botID,1)[0]
        self.a2_t = p.getJointState(self.botID,3)[0]
        self.b2_t = p.getJointState(self.botID,4)[0]
        self.w1_t = p.getJointState(self.botID,2)[0]
        self.w2_t = p.getJointState(self.botID,5)[0]

        #joint vel
        self.a1d_t = p.getJointState(self.botID,0)[1]
        self.b1d_t = p.getJointState(self.botID,1)[1]
        self.a2d_t = p.getJointState(self.botID,3)[1]
        self.b2d_t = p.getJointState(self.botID,4)[1]
        self.w1d_t = p.getJointState(self.botID,2)[1]
        self.w2d_t = p.getJointState(self.botID,5)[1]

        self.obs_t = [self.x_t, self.y_t, self.z_t, self.xd_t, self.yd_t, self.zd_t,
                        self.phi_t, self.the_t, self.shi_t, self.phid_t, self.thed_t, self.shid_t,
                        self.a1_t, self.b1_t, self.a2_t, self.b2_t, self.w1_t, self.w2_t,
                        self.a1d_t, self.b1d_t, self.a2d_t, self.b2d_t, self.w1d_t, self.w2d_t]
        
        return self.obs_t

    def action_t(self, a1,b1,w1,a2,b2,w2):
        p.setJointMotorControl2(self.botID, 0, p.POSITION_CONTROL, targetPosition = a1)
        p.setJointMotorControl2(self.botID, 1, p.POSITION_CONTROL, targetPosition = b1)
        p.setJointMotorControl2(self.botID, 2, p.VELOCITY_CONTROL, targetVelocity = w1, maxVelocity = 20)
        p.setJointMotorControl2(self.botID, 3, p.POSITION_CONTROL, targetPosition = a2)
        p.setJointMotorControl2(self.botID, 4, p.POSITION_CONTROL, targetPosition = b2)
        p.setJointMotorControl2(self.botID, 3, p.VELOCITY_CONTROL, targetVelocity = w2, maxVelocity = 20)
        
        return



