import os
from math import atan2, asin, sqrt
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg._quaternion import Quaternion

M_PI=3.1415926535

class Logger:
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        self.filename = filename
        os.makedirs(os.path.dirname(filename), exist_ok=True) # make directory if directory does not exist
        with open(self.filename, 'w') as file:
            header_str=""

            for header in headers:
                header_str+=header
                header_str+=", "
            
            header_str+="\n"
            
            file.write(header_str)


    def log_values(self, values_list):
        with open(self.filename, 'a') as file:
            vals_str = ', '.join(values_list)
            vals_str+="\n"
            file.write(vals_str)

class FileReader:
    def __init__(self, filename):
        self.filename = filename
        
    def read_file(self):
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:
            # Skip the header line

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


def euler_from_quaternion(quat: Quaternion):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    
    rotation = R.from_quat([quat.x, quat.y, quat.z, quat.w])
    euler_angles = rotation.as_euler('xyz', degrees=False)
    
    return euler_angles[2]


