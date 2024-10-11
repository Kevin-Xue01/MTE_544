# Imports
import rclpy

from rclpy.node import Node

from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.time import Time

CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']

class motion_executioner(Node):
    
    def __init__(self, motion_type=0):
        super().__init__("motion_types")
        
        self.type=motion_type
        self.linear_x = 0.0
        
        self.vel_publisher=self.create_publisher(Twist, "/cmd_vel", 10) # initialize velocity publisher     
        
        # initalize loggers
        self.imu_logger=Logger('logs/imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('logs/odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('logs/laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "angle_increment", "stamp"])
        
        qos=QoSProfile(reliability=2, durability=2, history=1, depth=10) # create QoS profile based on tutorial

        # imu subscription
        self.create_subscription(Imu, "/imu", self.imu_callback, qos_profile=qos)
        
        # encoder/odometry subscription
        self.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile=qos)

        
        # laser scan subscription 
        self.create_subscription(LaserScan, "/scan",self.laser_callback, qos_profile=qos)
        
        # Initialize timer with interval of 0.1 seconds to publish Twist msgs based on path type
        self.create_timer(0.1, self.timer_callback)

    def imu_callback(self, imu_msg: Imu):
        timestamp = str(Time.from_msg(imu_msg.header.stamp).nanoseconds) # extract timestamp in nanoseconds from the imu_msg object
        accel_x = str(imu_msg.linear_acceleration.x) # extract x acceleration from the imu_msg object
        accel_y = str(imu_msg.linear_acceleration.y) # extract y acceleration from the imu_msg object
        angular_z = str(imu_msg.angular_velocity.z) # extract angular veloctiy from the imu_msg object
        self.imu_logger.log_values([accel_x, accel_y, angular_z, timestamp]) # log the imu_msg data to csv
        
    def odom_callback(self, odom_msg: Odometry):
        timestamp = str(Time.from_msg(odom_msg.header.stamp).nanoseconds) # extract timestamp in nanoseconds from the odom_msg
        odom_yaw = str(euler_from_quaternion(odom_msg.pose.pose.orientation)) # extract orientation as a quaternion from the odom_msg object and convert to a yaw angle
        odom_x_pos = str(odom_msg.pose.pose.position.x) # extract x position from the odom_msg object
        odom_y_pos = str(odom_msg.pose.pose.position.y) # extract y position from the odom_msg object
        self.odom_logger.log_values([odom_x_pos, odom_y_pos, odom_yaw, timestamp]) # log the odom_msg data to csv
        
                
    def laser_callback(self, laser_msg: LaserScan):
        timestamp = Time.from_msg(laser_msg.header.stamp).nanoseconds # extract timestamp in nanoseconds from the laser_msg
        ranges = laser_msg.ranges # extract range data from the laser_msg object
        angle_increment = laser_msg.angle_increment # extract angle increment from the laser_msg object
        self.laser_logger.log_values(["|".join([str(i) for i in ranges]), str(angle_increment), str(timestamp)]) # log the laser_msg data to csv, convert range data of type=list to a string separated by '|'


    # publish Twist msg based on motion type   
    def timer_callback(self):
        cmd_vel_msg=Twist()
        
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()
                        
        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()
            
        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        self.vel_publisher.publish(cmd_vel_msg)
            
    def make_circular_twist(self):
        msg=Twist()

        # set a constant linear and angular velocity to achieve circular path
        msg.angular.z = 1.0
        msg.linear.x = 0.5

        return msg

    def make_spiral_twist(self):
        msg=Twist()
        msg.angular.z = 1.0        

        # increase velocity over time to achieve linear acceleration,
        # combined with non-zero angular velocity to achieve a spiral path
        msg.linear.x = self.linear_x
        self.linear_x += 0.01 

        return msg
    
    def make_acc_line_twist(self):
        msg=Twist()

        # increase velocity over time to achieve linear acceleration
        msg.linear.x = self.linear_x
        self.linear_x += 0.01 

        return msg

import argparse

if __name__=="__main__":
    argParser=argparse.ArgumentParser(description="input the motion type")

    argParser.add_argument("--motion", type=str, default="circle")

    rclpy.init()

    args = argParser.parse_args()

    if args.motion.lower() == "circle":
        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)
    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)
    else:
        print(f"we don't have {args.motion.lower()} motion type")
    
    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")
