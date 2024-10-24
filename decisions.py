import sys

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl

from rclpy import init, spin, spin_once
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom
import time

from localization import localization, rawSensor

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, PARABOLA, SIGMOID, planner
from controller import controller, trajectoryController

from rclpy.qos import QoSProfile, ReliabilityPolicy

# You may add any other imports you may need/want to use below
# import ...


class decision_maker(Node):
    
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, goalPoint, rate=10, motion_type=POINT_PLANNER):

        super().__init__("decision_maker")

        # Part 4: Create a publisher for the topic responsible for robot's motion
        self.publisher=self.create_publisher(publisher_msg, publishing_topic, qos_publisher)

        publishing_period=1/rate
        
        # Instantiate the controller
        # TODO Part 5: Tune your parameters here
    
        if motion_type == POINT_PLANNER:
            self.controller=controller(klp=0.2, klv=0.5, kap=0.8, kav=0.6)
            self.planner=planner(POINT_PLANNER, None)    
    
    
        elif motion_type==TRAJECTORY_PLANNER:
            self.controller=trajectoryController(klp=0.2, klv=0.5, kap=0.8, kav=0.6)
            self.planner=planner(TRAJECTORY_PLANNER, PARABOLA) # Can change to sigmoid later

        else:
            print("Error! you don't have this planner", file=sys.stderr)


        # Instantiate the localization, use rawSensor for now  
        self.localizer=localization(rawSensor)

        # Instantiate the planner
        # NOTE: goalPoint is used only for the pointPlanner
        self.goal=self.planner.plan(goalPoint)

        self.callback_timer = self.create_timer(publishing_period, self.timerCallback)


    def timerCallback(self):        
        # Part 3: Run the localization node
        # Remember that this file is already running the decision_maker node.
        spin_once(self.localizer)

        pose = self.localizer.getPose()

        if pose is None:
            print("waiting for odom msgs ....")
            return

        vel_msg=Twist()
        
        # Part 3: Check if you reached the goal
        if type(self.goal) == tuple:
            reached_goal = (calculate_linear_error(pose, self.goal) < 0.01)
        else: 
            reached_goal = False
        

        if reached_goal:
            print("Reached goal")
            self.publisher.publish(vel_msg)
            
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()
            
            # Part 3: exit the spin
            # cancel the timer because our jobs done
            self.callback_timer.cancel()
             
        
        velocity, yaw_rate = self.controller.vel_request(self.localizer.getPose(), self.goal, True)

        # Part 4: Publish the velocity to move the robot
        # Print velocity and yaw rate
        # commanded_velocities = Twist()
        vel_msg.linear.x = velocity
        vel_msg.angular.z = yaw_rate
        print("Linear Velocity: {}, Angular Velocity: {}".format(vel_msg.linear.x, vel_msg.angular.z))
        print("Pose: {}".format(self.localizer.getPose()))
        self.publisher.publish(vel_msg)

import argparse


def main(args=None):
    
    init()

    if args is None: 
        print("invalid args", file=sys.stderr)        
        return 

    # Part 3: You migh need to change the QoS profile based on whether you're using the real robot or in simulation.
    # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3
    
    # odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)
    
    odom_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,  # Ensures all messages are delivered
        depth=10  # Sets the queue size for outgoing messages
    )
    # Part 4: instantiate the decision_maker with the proper parameters for moving the robot
    if args.motion.lower() == "point":
        DM=decision_maker(Twist, "cmd_vel", odom_qos, goalPoint=[5.0, 5.0])
    elif args.motion.lower() == "trajectory":
        DM=decision_maker(Twist, "cmd_vel", odom_qos, goalPoint=[5.0, 5.0])
    else:
        print("invalid motion type", file=sys.stderr)        
        return 
    
    
    
    try:
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")


if __name__=="__main__":

    argParser=argparse.ArgumentParser(description="point or trajectory") 
    argParser.add_argument("--motion", type=str, default="point")
    args = argParser.parse_args()

    main(args)
