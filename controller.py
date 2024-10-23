import numpy as np

from pid import PID_ctrl
from utilities import Config, calculate_angular_error, calculate_linear_error

class controller:
    def __init__(self):
        self.PID_linear=PID_ctrl(Config.LINEAR_CONTROLLER_GAIN, filename_="linear.csv")
        self.PID_angular=PID_ctrl(Config.ANGULAR_CONTROLLER_GAIN, filename_="angular.csv")
    
    def vel_request(self, pose, goal, status):
        
        e_lin=calculate_linear_error(pose, goal)
        e_ang=calculate_angular_error(pose, goal)

        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status)
        
        if(abs(linear_vel) > Config.LINEAR_VELOCITY_LIMIT):
            if(linear_vel>0):
                linear_vel = Config.LINEAR_VELOCITY_LIMIT
            else:
                linear_vel = - Config.LINEAR_VELOCITY_LIMIT

        if(abs(angular_vel) > Config.ANGULAR_VELOCITY_LIMIT):
            if(angular_vel>0):
                angular_vel = Config.ANGULAR_VELOCITY_LIMIT
            else:
                angular_vel = -Config.ANGULAR_VELOCITY_LIMIT
        
        return linear_vel, angular_vel
    

class trajectoryController(controller):
    def __init__(self):
        super().__init__()
    
    def vel_request(self, pose, listGoals, status):
        goal=self.lookFarFor(pose, listGoals)
        
        finalGoal=listGoals[-1]
        
        e_lin=calculate_linear_error(pose, finalGoal)
        e_ang=calculate_angular_error(pose, goal)

        
        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status) 

        if(abs(linear_vel) > Config.LINEAR_VELOCITY_LIMIT):
            linear_vel = Config.LINEAR_VELOCITY_LIMIT if linear_vel > 0 else (-Config.LINEAR_VELOCITY_LIMIT)

        if(abs(angular_vel) > Config.ANGULAR_VELOCITY_LIMIT):
            angular_vel = Config.ANGULAR_VELOCITY_LIMIT if angular_vel > 0 else (-Config.ANGULAR_VELOCITY_LIMIT)
        
        return linear_vel, angular_vel

    def lookFarFor(self, pose, listGoals):
        poseArray=np.array([pose[0], pose[1]]) 
        listGoalsArray=np.array(listGoals)

        distanceSquared=np.sum((listGoalsArray-poseArray)**2,
                               axis=1)
        closestIndex=np.argmin(distanceSquared)

        return listGoals[ min(closestIndex + 3, len(listGoals) - 1) ]
