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

