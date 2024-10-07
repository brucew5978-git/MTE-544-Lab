# Imports
import rclpy

from rclpy.node import Node

from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile

# TODO Part 3: Import message types needed: 
    # For sending velocity commands to the robot: Twist
    # For the sensors: Imu, LaserScan, and Odometry
# Check the online documentation to fill in the lines below
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.time import Time

# You may add any other imports you may need/want to use below
# import ...


CIRCLE=0; SPIRAL=1; ACC_LINE=2; STOP=3
MAX_SPEED_GAIN = 1.5
motion_types=['circle', 'spiral', 'line']

class motion_executioner(Node):
    
    def __init__(self, motion_type=0):
        
        super().__init__("motion_types")
        
        self.type=motion_type
        
        self.radius_=0.0
        
        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False

        self.speed_gain = 0
        
        # TODO Part 3: Create a publisher to send velocity commands by setting the proper parameters in (...)
        self.vel_publisher=self.create_publisher(Twist, 'cmd_vel', 10)
                
        # loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "angle_increment", "stamp"])
        
        # TODO Part 3: Create the QoS profile by setting the proper parameters in (...)
        qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)

        # TODO Part 5: Create below the subscription to the topics corresponding to the respective sensors
        # IMU subscription
        self.imu_sub = self.create_subscription(Imu, "imu", self.imu_callback, qos_profile=qos)
       

        
        # ENCODER subscription # TODO fix this
        self.enc_sub = self.create_subscription(Odometry, "odom", self.odom_callback, qos_profile=qos)
        self.odom_initialized=True

        
        # LaserScan subscription 
        self.lidar_sub = self.create_subscription(LaserScan, "scan", self.laser_callback, qos_profile=qos)
        self.laser_initialized=True
        
        self.create_timer(0.1, self.timer_callback)
        self.successful_init=True
        print("inited")


    # TODO Part 5: Callback functions: complete the callback functions of the three sensors to log the proper data.
    # To also log the time you need to use the rclpy Time class, each ros msg will come with a header, and then
    # inside the header you have a stamp that has the time in seconds and nanoseconds, you should log it in nanoseconds as 
    # such: Time.from_msg(imu_msg.header.stamp).nanoseconds
    # You can save the needed fields into a list, and pass the list to the log_values function in utilities.py

    def imu_callback(self, imu_msg: Imu):
        val_list = [imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.angular_velocity.z, Time.from_msg(imu_msg.header.stamp).nanoseconds]
        self.imu_logger.log_values(val_list)
        
    def odom_callback(self, odom_msg: Odometry):
        val_list = [odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.angular.z, Time.from_msg(odom_msg.header.stamp).nanoseconds]
        self.odom_logger.log_values(val_list)
                
    def laser_callback(self, laser_msg: LaserScan):
        # log laser msgs with position msg at that time
        val_list = []
        for range in laser_msg.ranges:
            if  range > laser_msg.range_min and range < laser_msg.range_max : 
                val_list.append(range)

        val_list.append(laser_msg.angle_increment)
        val_list.append(Time.from_msg(laser_msg.header.stamp).nanoseconds)
        self.laser_logger.log_values(val_list)
    
   
        
                
    def timer_callback(self):
        print("timer callback")
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            print("test")
            self.successful_init=True
            
        if not self.successful_init:
            return
        
        cmd_vel_msg=Twist()
        
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()
                        
        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()

        elif self.type==STOP:
            cmd_vel_msg=self.stop_robot()
            
        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        print("publishing")
        self.vel_publisher.publish(cmd_vel_msg)
        
    
    # TODO Part 4: Motion functions: complete the functions to generate the proper messages corresponding to the desired motions of the robot

    def make_circular_twist(self):
        
        msg=Twist()
        # fill up the twist msg for circular motion
        msg.linear.x = 0.1
        msg.angular.z = 0.3
        return msg

    def make_spiral_twist(self):
        msg=Twist()
        # fill up the twist msg for spiral motion
        msg.linear.x = 0.2
        msg.angular.z = 1.5 - self.speed_gain

        if (self.speed_gain < MAX_SPEED_GAIN):
            self.speed_gain += 0.005
        return msg
    
    def make_acc_line_twist(self):
        msg=Twist()
        # fill up the twist msg for line motion
        msg.linear.x = 0.2
        msg.angular.z = 0.0
        return msg
 
    def stop_robot(self): 
        msg=Twist()
        # fill up the twist msg for line motion
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        return msg
        

import argparse

if __name__=="__main__":
    

    argParser=argparse.ArgumentParser(description="input the motion type")


    argParser.add_argument("--motion", type=str, default="circle")

    print("before init")
    rclpy.init()

    args = argParser.parse_args()

    if args.motion.lower() == "circle":

        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)

    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)
    else:
        ME=motion_executioner(motion_type=STOP)
        print(f"we don't have {args.motion.lower()} motion type")


    
    try:
        print("spinning")
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")
