#!/usr/bin/env python3

import os
import csv
import math
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDrive

from hybrid_astar_planner import HybridAStarPlanner  # import your planner

traffic_sign_detected = None

def traffic_sign_callback(msg):
    global traffic_sign_detected
    traffic_sign_detected = msg.data
    rospy.loginfo(f"Received traffic sign: {traffic_sign_detected}")

class PurePursuitHybrid:
    def __init__(self):
        rospy.init_node('vicon_pp_node', anonymous=True)
        rospy.Subscriber('/yolo/traffic_sign', String, traffic_sign_callback)

        self.rate = rospy.Rate(50)

        self.look_ahead = 0.2
        self.wheelbase = 0.325
        self.offset = 0.0

        self.ctrl_pub = rospy.Publisher('/car_1/offboard/command', AckermannDrive, queue_size=1)
        self.drive_msg = AckermannDrive()
        self.drive_msg.speed = 0.3

        self.gt_sub = rospy.Subscriber('/car_1/ground_truth', Odometry, self.carstate_callback_gt)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.read_waypoints()
        self.goal = 0

        self.occ_grid = np.zeros((200, 200), dtype=bool)
        self.hp = HybridAStarPlanner(self.occ_grid, step_size=0.5, angles=[-0.4, 0, 0.4], R=1.2)

    def carstate_callback_gt(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.yaw = np.degrees(euler[2])

    def read_waypoints(self):
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, '../waypoints/xyhead_demo_pp.csv')
        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]
        self.path_points_x = np.array([float(p[0]) for p in path_points])
        self.path_points_y = np.array([float(p[1]) for p in path_points])
        self.path_points_yaw = np.array([float(p[2]) for p in path_points])
        self.wp_size = len(self.path_points_x)
        self.dist_arr = np.zeros(self.wp_size)

    def get_state(self):
        curr_yaw = np.radians(self.yaw)
        curr_x = self.x - self.offset * np.cos(curr_yaw)
        curr_y = self.y - self.offset * np.sin(curr_yaw)
        return round(curr_x, 3), round(curr_y, 3), round(curr_yaw, 4)

    def dist(self, p1, p2):
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def is_collision(self, x, y):
        ix, iy = int(round(x)), int(round(y))
        if ix < 0 or iy < 0 or ix >= self.hp.w or iy >= self.hp.h:
            return True
        return self.hp.occ[iy, ix]

    def collision_between(self, start, goal, num_samples=20):
        x0, y0, _ = start
        x1, y1, _ = goal
        for i in range(1, num_samples + 1):
            alpha = i / num_samples
            xi = x0 + alpha * (x1 - x0)
            yi = y0 + alpha * (y1 - y0)
            if self.is_collision(xi, yi):
                return True
        return False

    def start_pp(self):
        global traffic_sign_detected

        while not rospy.is_shutdown():
            curr_x, curr_y, curr_yaw = self.get_state()

            for i in range(self.wp_size):
                self.dist_arr[i] = self.dist((self.path_points_x[i], self.path_points_y[i]), (curr_x, curr_y))

            goal_arr = np.where((self.dist_arr < self.look_ahead + 0.1) & (self.dist_arr > self.look_ahead - 0.1))[0]

            if len(goal_arr) == 0:
                rospy.logwarn("No valid goal found.")
                self.rate.sleep()
                continue

            self.goal = goal_arr[-1]  # pick the furthest

            car_pose = (curr_x, curr_y, curr_yaw)
            goal_pose = (self.path_points_x[self.goal], self.path_points_y[self.goal], np.radians(self.path_points_yaw[self.goal]))

            if self.collision_between(car_pose, goal_pose):
                rospy.loginfo("Obstacle between car and goal. Replanning...")
                detour = self.hp.plan(car_pose, goal_pose)
                if detour:
                    next_x, next_y, next_yaw = detour[min(3, len(detour)-1)]
                    dx = next_x - curr_x
                    dy = next_y - curr_y
                    desired_heading = math.atan2(dy, dx)
                    L = np.hypot(dx, dy)
                else:
                    rospy.logwarn("Hybrid A* failed.")
                    self.drive_msg.speed = 0.0
                    self.ctrl_pub.publish(self.drive_msg)
                    continue
            else:
                dx = goal_pose[0] - curr_x
                dy = goal_pose[1] - curr_y
                desired_heading = math.atan2(dy, dx)
                L = self.dist_arr[self.goal]

            alpha = desired_heading - curr_yaw

            # Curvature control
            k = 0.2
            angle_i = math.atan((k * 2 * self.wheelbase * math.sin(alpha)) / L)
            angle = angle_i * 2
            f_delta = np.clip(angle, -0.3, 0.3)

            if traffic_sign_detected == 'stop sign':
                self.drive_msg.speed = 0.0
                rospy.loginfo("Stopping due to stop sign.")
            else:
                self.drive_msg.speed = 0.3

            self.drive_msg.steering_angle = -f_delta
            self.ctrl_pub.publish(self.drive_msg)
            self.rate.sleep()

def pure_pursuit():
    pp = PurePursuitHybrid()
    try:
        pp.start_pp()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    pure_pursuit()
