import os
from math import atan2, asin, sqrt, atan
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg._quaternion import Quaternion
from enum import Enum

class ControllerType(Enum):
    P = 'P'
    PD = 'PD'
    PI = 'PI'
    PID = 'PID'

class PlannerType(Enum):
    POINT = 'POINT'
    TRAJECTORY = 'TRAJECTORY'
    PATH_PARABOLA = 'PARABOLA'
    PATH_SIGMOID = 'SIGMOID'

class ControllerGains(Enum):
    KP = 'KP'
    KI = 'KI'
    KD = 'KD'

class Config:
    X_0 = 0.0
    Y_0 = 0.0
    X_F = 1.5
    Y_F = -2.0

    LINEAR_VELOCITY_LIMIT = 1.0
    ANGULAR_VELOCITY_LIMIT = 1.0

    CONTROLLER_TYPE = ControllerType.P

    LINEAR_CONTROLLER_GAIN = {
        ControllerGains.KP: 1.2,
        ControllerGains.KI: 0.2,
        ControllerGains.KD: 0.8
    }
    ANGULAR_CONTROLLER_GAIN = {
        ControllerGains.KP: 1.2,
        ControllerGains.KI: 0.2,
        ControllerGains.KD: 0.8
    }

    PLANNER_PATH = PlannerType.PATH_PARABOLA
    PLANNER_TRAJECTORY_RESOLUTION = 100
    PLANNER_TRAJECTORY_GAIN = 1.0
    PLANNER_TRAJECTORY_XMAX = 2.5


class Logger:
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        self.filename = filename

        with open(self.filename, 'w') as file:
            header_str = ",".join(headers) + "\n"
            file.write(header_str)


    def log_values(self, values_list: list[str]):
        with open(self.filename, 'a') as file:
            vals_str = ",".join(values_list) + "\n"
            file.write(vals_str)

    def save_log(self):
        pass

class FileReader:
    def __init__(self, filename):
        self.filename = filename
        
    def read_file(self):
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:

            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break
            
            next(file)
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row=[]                
                
                for val in values:
                    if val=='':
                        break
                    row.append(float(val.strip()))

                table.append(row)
        
        return headers, table
    
    

# TODO Part 3: Implement the conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """

    rotation = R.from_quat([quat.x, quat.y, quat.z, quat.w])
    euler_angles = rotation.as_euler('xyz', degrees=False)
    
    return euler_angles[2]


def calculate_linear_error(current_pose, goal_pose):
    x_diff = goal_pose[0] - float(current_pose[0])
    y_diff = goal_pose[1] - float(current_pose[1])
    
    error_linear= sqrt((x_diff*x_diff)+(y_diff*y_diff))
    print("linear diff:", error_linear)

    return error_linear

def calculate_angular_error(current_pose, goal_pose):
    x_diff = goal_pose[0] - float(current_pose[0])
    y_diff = goal_pose[1] - float(current_pose[1])

    error_angular = atan2(y_diff,x_diff) - float(current_pose[2]) # range = [-π, π]

    return error_angular