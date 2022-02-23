class Trajectory:
    def __init__(self,path):
        self.file = path
        self.num_com = len(self.file)
        self.curr_com = 0
        self.time = 0        
        self.file_ptr = 0
        # file --> |  t  |  lin_x  |  rot_z  |
        #          |  0  |    12   |    5    |

        self.file_time = 0
        self.lin_vel = 0
        self.rot_vel = 0

        return
    
    def get_cmd_vel(self,timestep):

        for i in range(len(self.file)):
            if timestep <= self.file[i][0]:
                self.lin_vel = self.file[i-1][1]
                self.rot_vel = self.file[i-1][2]
        '''
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
        '''
        '''
        if self.curr_com != self.num_com:
            if timestep >= self.file[self.file_ptr][0] and timestep < self.file[self.file_ptr+1][0]:
                self.lin_vel = self.file[self.file_ptr][1]
                self.rot_vel = self.file[self.file_ptr][2]
            else:
                self.file_ptr = self.file_ptr + 1
                self.curr_com = self.curr_com + 1
                self.lin_vel = self.file[self.file_ptr][1]
                self.rot_vel = self.file[self.file_ptr][2]
        else:
            self.lin_vel = self.file[self.file_ptr][1]
            self.rot_vel = self.file[self.file_ptr][2]
        '''

        return [self.lin_vel, self.rot_vel]