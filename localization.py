import sys
import rclpy
from datetime import datetime
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom
from rclpy import init, spin

from utilities import Logger, euler_from_quaternion

rawSensor = 0
class localization(Node):
    
    def __init__(self, localizationType=rawSensor):
        super().__init__("localizer")
        
        # TODO Part 3: Define the QoS profile variable based on whether you are using the simulation (Turtlebot 3 Burger) or the real robot (Turtlebot 4)
        # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3
        odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10) # create QoS profile based on tutorial
        
        self.loc_logger=Logger("robot_pose.csv", headers=["x", "y", "theta", "stamp"])
        self.pose = (0.0, 0.0, 0.0, int(datetime.now()))
        
        if localizationType == rawSensor:
            self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)
        else:
            raise SystemError("This type doesn't exist", sys.stderr)
    
    
    def odom_callback(self, pose_msg):
        timestamp = int(Time.from_msg(pose_msg.header.stamp).nanoseconds) # extract timestamp in nanoseconds from the odom_msg
        odom_yaw = float(euler_from_quaternion(pose_msg.pose.pose.orientation)) # extract orientation as a quaternion from the odom_msg object and convert to a yaw angle
        odom_x_pos = float(pose_msg.pose.pose.position.x) # extract x position from the odom_msg object
        odom_y_pos = float(pose_msg.pose.pose.position.y) # extract y position from the odom_msg object

        self.pose = (odom_x_pos, odom_y_pos, odom_yaw, timestamp)
        
        self.loc_logger.log_values(self.pose)

    def getPose(self) -> tuple[float, float, float, int]:
        return self.pose

def main(args = None):
    rclpy.init()

    LC = localization(localizationType=rawSensor)

    try:
        rclpy.spin(LC)
    except KeyboardInterrupt:
        print("Exiting")


if __name__=="__main__":
    main()