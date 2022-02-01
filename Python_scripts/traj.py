class Trajectory:
    def __init__(self,path):
        self.file = path
        self.time = 0        
        self.file_ptr = 0
        # file --> |  t  |  lin_x  |  rot_z  |
        #          |  0  |    12   |    5    |

        self.file_time = 0
        self.lin_vel = 0
        self.rot_vel = 0

        return
    
    def get_cmd_vel(self,timestep):
        try:
            if timestep >= self.file[self.file_ptr][0] and timestep < self.file[self.file_ptr+1][0]:
                self.lin_vel = self.file[self.file_ptr][1]
                self.rot_vel = self.file[self.file_ptr][2]
            else:
                self.file_ptr = self.file_ptr + 1
                self.lin_vel = self.file[self.file_ptr][1]
                self.rot_vel = self.file[self.file_ptr][2]

        except:
            self.lin_vel = self.file[self.file_ptr][1]
            self.rot_vel = self.file[self.file_ptr][2]

        return [self.lin_vel, self.rot_vel]