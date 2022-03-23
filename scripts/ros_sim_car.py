#!/usr/bin/env python
import rospy 
import numpy as np
import time
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

import tf 
import cubic_spline_planner

from rviz_clock_plugin.msg import ETA

k = 0.5  # control gain
Kp = 1.0  # speed proportional gain
dt = 0.05  # [s] time difference
L = 2.9  # [m] Wheel base of vehicle
max_steer = np.radians(40.0)  # [rad] max steering angle

def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

def pid_control(target, current):
    """
    Proportional control for the speed.

    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    return Kp * (target - current)

def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.

    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # Calc front axle position
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                      -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle

def stanley_control(state, cx, cy, cyaw, last_target_idx):
    """
    Stanley steering control.

    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    # Steering control
    delta = theta_e + theta_d

    return delta, current_target_idx

class State(object):
    """
    Class representing the state of a vehicle.

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super(State, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.pose = PoseStamped()
        self.pose_pub = rospy.Publisher("robot_pose", PoseStamped, queue_size = 1)
        # self.pub_pose()
        self.timer_start = 0.0

    def update(self, acceleration, delta):
        """
        Update the state of the vehicle.

        Stanley Control uses bicycle model.

        :param acceleration: (float) Acceleration
        :param delta: (float) Steering
        """
        delta = np.clip(delta, -max_steer, max_steer)

        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / L * np.tan(delta) * dt
        self.yaw = normalize_angle(self.yaw)
        self.v += acceleration * dt

        # self.pub_pose()

    def pub_pose(self):
        self.pose.header.frame_id = "map"
        self.pose.header.stamp = rospy.Time.now()
        self.pose.pose.position.x = self.x
        self.pose.pose.position.y = self.y 

        q = tf.transformations.quaternion_from_euler(0,0,self.yaw)
        self.pose.pose.orientation.x = q[0]
        self.pose.pose.orientation.y = q[1]
        self.pose.pose.orientation.z = q[2]
        self.pose.pose.orientation.w = q[3]
        
        self.pose_pub.publish(self.pose)
        

class FakeRobot():

    def __init__(self):

        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callbackGoal, queue_size=1)
        self.path_pub = rospy.Publisher("global_path", Path, queue_size = 1)
        self.eta_pub = rospy.Publisher("/eta", ETA, queue_size = 1)
        self.goal = PoseStamped()
        self.cx = []
        self.cy = []
        self.cyaw = []
        self.s = []
        self.path = Path()
        self.reset_path = False
        self.rate = rospy.Rate(1.0/dt)
        self.target_idx = 0
        self.target_speed = 1.0
        self.last_idx = 0
        self.state = State()
        self.total_distance = 0
        self.eta_msg = ETA()

    def callbackGoal(self, msg):
        self.path = Path()
        self.goal = msg
        step =5.0
        x_step = (self.goal.pose.position.x - self.state.x)/step
        y_step = (self.goal.pose.position.y - self.state.y)/step
        noise = np.random.normal(0, .1, int(step))
        noise[0]=0
        ax = np.arange(self.state.x, self.goal.pose.position.x, x_step) + noise
        ay = np.arange(self.state.y, self.goal.pose.position.y, y_step) + noise

        ax = np.append(ax, self.goal.pose.position.x)
        ay = np.append(ay, self.goal.pose.position.y)
        
        self.cx, self.cy, self.cyaw, _, self.s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)
        self.total_distance = self.s[-1]

        self.path.header.frame_id = "map"
        timestamp = rospy.Time.now()
        self.path.header.stamp = timestamp
        for i in range(len(self.cx)):            
            pose = PoseStamped()
            pose.header.stamp = timestamp
            pose.header.frame_id = "map"
            pose.pose.position.x = self.cx[i]
            pose.pose.position.y = self.cy[i]
            q = tf.transformations.quaternion_from_euler(0,0,self.cyaw[i])
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            self.path.poses.append(pose)
        self.path_pub.publish(self.path)
        self.reset_path = True

    def calculate_eta(self, stop=False):
        if stop:
            if self.eta_msg.status == ETA.STATUS_EXECUTING:
                self.eta_msg.status = ETA.STATUS_PENDING
        else:
            self.eta_msg.status = ETA.STATUS_EXECUTING
        distance_to_next_target_idx = np.sqrt((self.cx[self.target_idx] - self.state.x)**2 + (self.cy[self.target_idx] - self.state.y)**2)
        remain_distance = distance_to_next_target_idx + self.s[-1] - self.s[self.target_idx]
        estimate_time = (remain_distance)/self.target_speed
        # print("total distance: ", self.total_distance, "remain: distance: ", remain_distance, "ETA: ", estimate_time, " s")
        self.eta_msg.time = estimate_time

        ## add a stop at every modulo 10 seconds
        if self.eta_msg.time % 10 < 0.03:
            self.eta_msg.status = ETA.STATUS_PENDING
            self.eta_pub.publish(self.eta_msg)
            time.sleep(2)
            
        self.eta_pub.publish(self.eta_msg)

    def stop(self, arrived=True):
        if arrived:
            self.eta_msg.status = ETA.STATUS_FINISHED
            self.eta_msg.time = 0.0
            self.eta_pub.publish(self.eta_msg)
        else:
            self.calculate_eta(stop=True)

    def track(self):
        if len(self.cx) == 0:
            return

        if self.reset_path:
            self.last_idx = len(self.cx) -1
            self.target_idx, _ = calc_target_index(self.state, self.cx, self.cy)
            self.reset_path = False
        
        distance_to_goal = np.hypot(self.state.x - self.goal.pose.position.x, self.state.y - self.goal.pose.position.y)
        if distance_to_goal > 0.2 :
            self.calculate_eta(stop=False)
            ai = pid_control(self.target_speed, self.state.v)
            di, self.target_idx = stanley_control(self.state, self.cx, self.cy, self.cyaw, self.target_idx)
            self.state.update(ai, di)
        else:
            self.stop()
        
    def run(self):
        while not rospy.is_shutdown():
            self.track()
            self.state.pub_pose()
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node("ros_sim_car")

    fr = FakeRobot()
    try:
        fr.run()
    except rospy.ROSInterruptException: pass