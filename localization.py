import sys
import rclpy

from utilities import Logger, euler_from_quaternion
from rclpy.time import Time
from rclpy.node import Node

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from rclpy import init, spin

rawSensor = 0
class localization(Node):
    
    def __init__(self, localizationType=rawSensor):

        super().__init__("localizer")
        
        # TODO Part 3: Define the QoS profile variable based on whether you are using the simulation (Turtlebot 3 Burger) or the real robot (Turtlebot 4)
        # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3

        odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10) # create QoS profile based on tutorial
        
        self.loc_logger=Logger("robot_pose.csv", ["x", "y", "theta", "stamp"])
        self.pose=None
        
        if localizationType == rawSensor:
        # TODO Part 3: subscribe to the position sensor topic (Odometry)
            self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)
        else:
            print("This type doesn't exist", sys.stderr)
    
    
    def odom_callback(self, pose_msg):
        timestamp = str(Time.from_msg(pose_msg.header.stamp).nanoseconds) # extract timestamp in nanoseconds from the odom_msg
        odom_yaw = str(euler_from_quaternion(pose_msg.pose.pose.orientation)) # extract orientation as a quaternion from the odom_msg object and convert to a yaw angle
        odom_x_pos = str(pose_msg.pose.pose.position.x) # extract x position from the odom_msg object
        odom_y_pos = str(pose_msg.pose.pose.position.y) # extract y position from the odom_msg object
        
        # TODO Part 3: Read x,y, theta, and record the stamp
        self.pose=[odom_x_pos,odom_y_pos, odom_yaw, timestamp]
        
        # Log the data
        self.loc_logger.log_values([self.pose[0], self.pose[1], self.pose[2], Time.from_msg(self.pose[3]).nanoseconds])
    
    def getPose(self):
        return self.pose

    # TODO Part 3
    # Here put a guard that makes the node run, ONLY when run as a main thread!
    # This is to make sure this node functions right before using it in decision.py

    def main(args = None):
        rclpy.init()

        LC=localization(localizationType=rawSensor)

        try:
            rclpy.spin(LC)
        except KeyboardInterrupt:
            print("Exiting")


    if __name__=="__main__":
        main()
    
