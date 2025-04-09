import rospy
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import math
import time
from nav_msgs.msg import Path
import cv2

# global target_velocity_global
# target_velocity_global = 0.1

class vehicleController():

    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
        # self.waypoints = rospy.Subscriber("/waypoints/birdseye", Path, self.waypoint_callback, queue_size=1)
        self.sub_perception_error = rospy.Subscriber("/waypoints/error", Float32, self.perception_error_callback, queue_size=1)
        # self.sub_lidar_error = rospy.Subscriber("/tunnel_error", Float32, self.lidar_error_callback, queue_size=1)


        self.prev_vel = 0
        self.prev_yaw = 0
        self.L = 1.75  # Wheelbase
        self.log_acceleration = True
        self.log_debug = True
        self.acceleration = 0
        self.acceleration_log = []
        self.time_log = []
        self.x_log = []
        self.y_log = []
        self.start_time = time.time()
        self.waypoints_birdseye = []
        self.worldpoints = []
        self.imagepoints = []
        self.camera_error = None
     
        # PID control variables
        # for 0.03 velocity

        #TODO: Change parameters
        self.KP =
        self.KI = 
        self.KD = 
        self.MIN_ANGLE = -0.3
        self.MAX_ANGLE = 0.3
        self.SCALE_FACTOR = 
        self.velocity = 0.6

           
            
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None
        # To switch between adapative steering
        self.useAdapativevelocity = False

        # Waypoint settings
        self.lookaheaddist = 0.7
        self.offset = 25
        self.midpoint = 160

    # PID Control Method
    def pid_control(self, error, cur_time):
        # Initialize last_time on the first call
        if self.last_time is None:
            self.last_time = cur_time
            return 0.0

        # Calculate time difference
        delta_time = cur_time - self.last_time

        # Scale error term
        error /= self.SCALE_FACTOR

        # Calculate PID terms
        p_term = self.KP * error
        self.integral += error * delta_time
        i_term = self.KI * self.integral
        d_term = self.KD * (error - self.last_error) / delta_time

        # Calculate steering angle
        steering_angle = p_term + i_term + d_term

        # Constrain steering angle
        steering_angle = max(self.MIN_ANGLE, min(self.MAX_ANGLE, steering_angle))
                    
        # Update last error and last time
        self.last_error = error
        self.last_time = cur_time

        return steering_angle

    def execute(self):
        error_for_pid = None
        target_velocity = self.velocity
        target_steering = 0.0

        # Use the PID controller to determine steering angle based on the current error
        if self.camera_error is not None:
            cur_time = time.time()
            error_for_pid = self.camera_error
            if error_for_pid is not None:
                target_steering = self.pid_control(error_for_pid, cur_time)
        else:
            target_steering = 0.0  # Default steering if no error


        if (self.useAdapativevelocity):
            sharpness = min(abs(target_steering) / self.MAX_ANGLE, 1)
            adaptivespeed = target_velocity - (target_velocity - 0.45) * sharpness
            # Update vehicle state
            speed = adaptivespeed
        else:
            speed = target_velocity

        # logs
        if self.log_debug:
            print(f"Error: {error_for_pid}, Steering Angle: {target_steering}, Speed: {speed}")

        self.prev_vel = speed
        self.prev_yaw = target_steering

        # Pack computed velocity and steering angle into Ackermann command
        newAckermannCmd = AckermannDriveStamped()
        newAckermannCmd.drive.speed = speed
        newAckermannCmd.drive.steering_angle = target_steering

        # Publish the computed control input to vehicle model
        self.controlPub.publish(newAckermannCmd)

    def stop(self):
        newAckermannCmd = AckermannDriveStamped()
        newAckermannCmd.drive.speed = 0
        self.controlPub.publish(newAckermannCmd)

    def waypoint_callback(self,data):
        try:
            wp_originial = data
            length = len(wp_originial.poses)
            point = length - round(length * self.lookaheaddist)
            wp = wp_originial.poses[point]
            x_wp= wp.pose.position.x
            self.camera_error = self.midpoint + self.offset -(x_wp) 
            print(f'camera_error --------- {str(self.camera_error)}')

        except (ValueError,RuntimeError, IndexError):
            print('No waypoints generated')

    # Perception error
    def perception_error_callback(self, data):
        self.camera_error = data.data
        # print(f'Camera_error --------- {str(self.camera_error )}')
    
    def setImagepoints(self,data):
        self.imagepoints = data

    def setWorldpoints(self,data):
        self.worldpoints = data

    def getImagepoints(self):
        return self.imagepoints

    def getWorldpoints(self):
        return self.worldpoints

    def SetWaypoints(self,data):
        # print(data)
        self.waypoints_birdseye = data

    def GetWaypoints(self):
        return self.waypoints_birdseye
