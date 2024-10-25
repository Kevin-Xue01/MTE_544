import argparse
import sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
import rclpy
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from utilities import PlannerType, euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl
from localization import localization, rawSensor
from planner import planner
from controller import controller, trajectoryController

class decision_maker(Node):
    
    def __init__(self, publisher_msg, publishing_topic: str, qos_publisher, goalPoint, motion_type: PlannerType, rate=10):
        super().__init__("decision_maker")

        self.publisher = self.create_publisher(publisher_msg, publishing_topic, 10) # initialize velocity publisher

        publishing_period = 1 / rate
        
        self.controller = controller()
        self.planner = planner(motion_type)    
    
        # Instantiate the localization, use rawSensor for now  
        self.localizer = localization(rawSensor)

        # Instantiate the planner. NOTE: goalPoint is used only for the pointPlanner
        self.goal = self.planner.plan(goalPoint)

        self.create_timer(publishing_period, self.timerCallback)


    def timerCallback(self):
        rclpy.spin_once(self.localizer, timeout_sec=0.1)

        if self.localizer.getPose()  is  None:
            print("waiting for odom msgs ....")
            return

        vel_msg = Twist()
        
        # TODO Part 3: Check if you reached the goal
        if type(self.goal) == list:
            if(calculate_linear_error(self.localizer.getPose(), self.goal[-1]) < 0.1):
                reached_goal = True
            else:
                reached_goal = False
        else: 
            if(calculate_linear_error(self.localizer.getPose(), self.goal) < 0.1): #Robot is close enough to point
                reached_goal = True
            else:
                reached_goal = False
        

        if reached_goal:
            print("reached goal")
            self.publisher.publish(vel_msg)
            
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()
            
            raise SystemExit("Goal has been reached")
        
        velocity, yaw_rate = self.controller.vel_request(self.localizer.getPose(), self.goal, True)
        vel_msg.linear.x = velocity
        vel_msg.angular.z = yaw_rate

        #TODO Part 4: Publish the velocity to move the robot
        self.publisher.publish(vel_msg)



def main(args=None):
    rclpy.init()

    # TODO Part 3: You migh need to change the QoS profile based on whether you're using the real robot or in simulation.
    # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3
    
    qos = QoSProfile(reliability=2, durability=2, history=1, depth=10)
    publisher_msg = Twist
    publishing_topic = "/cmd_vel"
    destination_point = [1.5,-2] #random point in simulation

    if args.motion.lower() == "point":
        DM = decision_maker(publisher_msg, publishing_topic, qos,destination_point, PlannerType.POINT)
    elif args.motion.lower() == "trajectory":
        DM = decision_maker(publisher_msg, publishing_topic, qos,destination_point, PlannerType.TRAJECTORY)
    else:
        print("invalid motion type", file=sys.stderr)        

    try:
        rclpy.spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")
    rclpy.shutdown()

if __name__=="__main__":
    argParser=argparse.ArgumentParser(description="point or trajectory") 
    argParser.add_argument("--motion", type=str, default="point")
    args = argParser.parse_args()

    main(args)
