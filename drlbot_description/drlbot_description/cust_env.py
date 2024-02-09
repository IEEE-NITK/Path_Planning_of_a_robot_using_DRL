from gym import Env
from gym.spaces import Discrete, Box
import numpy as np
import random
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from squaternion import Quaternion
from std_srvs.srv import Empty
import matplotlib.pyplot as plt
import os
import time
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from numpy import inf
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import time
import numpy as np

import math
import random
from collections import deque

GOAL_REACHED_DIST = 0.3
COLLISION_DIST = 0.35
TIME_DELTA = 0.2

last_odom = None
environment_dim = 20

class ObstacleAvoidanceEnv(Env):
        
    def __init__(self):

        super().__init__('env')
        self.environment_dim = 20
        self.odom_x = 0
        self.odom_y = 0

        self.goal_x = 1
        self.goal_y = 0.0

        self.upper = 5.0
        self.lower = -5.0

        self.set_self_state = ModelState()
        #self.set_self_state.model_name = "r1"
        self.set_self_state.pose.position.x = 0.0
        self.set_self_state.pose.position.y = 0.0
        self.set_self_state.pose.position.z = 0.0
        self.set_self_state.pose.orientation.x = 0.0
        self.set_self_state.pose.orientation.y = 0.0
        self.set_self_state.pose.orientation.z = 0.0
        self.set_self_state.pose.orientation.w = 1.0

        # Set up the ROS publishers and subscribers
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.set_state = self.create_publisher(ModelState, "gazebo/set_model_state", 10)

        self.unpause = self.create_client(Empty, "/unpause_physics")
        self.pause = self.create_client(Empty, "/pause_physics")
        self.reset_proxy = self.create_client(Empty, "/reset_world")
        self.req = Empty.Request

        # Define observation and action spaces
        self.observation_space = Box(
            low=0, high=255, shape=(24,), dtype=np.uint8  # 360-degree lidar data + odometry + goal info
        )
        self.action_space = Box(low=-1, high=1, shape=(2,), dtype=np.float32)  # Steering, speed

        # Maximum episode length
        self.max_episode_steps = 5000

        # Initialize environment variables (replace with your simulation setup)
        # self.robot = ...  # Initialize robot object
        # self.lidar = ...  # Initialize lidar sensor

        # Reset the environment
        self.reset()

    def reset(self):

         # Resets the state of the environment and returns an initial observation.
        #rospy.wait_for_service("/gazebo/reset_world")
        while not self.reset_proxy.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('reset : service not available, waiting again...')

        try:
            self.reset_proxy.call_async(Empty.Request())
        except rclpy.ServiceException as e:
            print("/gazebo/reset_simulation service call failed")

        angle = np.random.uniform(-np.pi, np.pi)
        quaternion = Quaternion.from_euler(0.0, 0.0, angle)
        object_state = self.set_self_state

        x = 0
        y = 0
        position_ok = False
        while not position_ok:
            x = np.random.uniform(-4.5, 4.5)
            y = np.random.uniform(-4.5, 4.5)
            position_ok = check_pos(x, y)
        object_state.pose.position.x = x
        object_state.pose.position.y = y
        # object_state.pose.position.z = 0.
        object_state.pose.orientation.x = quaternion.x
        object_state.pose.orientation.y = quaternion.y
        object_state.pose.orientation.z = quaternion.z
        object_state.pose.orientation.w = quaternion.w
        self.set_state.publish(object_state)

        self.odom_x = object_state.pose.position.x
        self.odom_y = object_state.pose.position.y

        # Reset robot and sensor states (replace with your methods)
        self.step_count = 0
        # self.robot.reset()
        # self.lidar.reset()

        # Place a new goal and check if its location is not on one of the obstacles
        if self.upper < 10:
            self.upper += 0.004
        if self.lower > -10:
            self.lower -= 0.004

        goal_ok = False

        while not goal_ok:
            self.goal_x = self.odom_x + random.uniform(self.upper, self.lower)
            self.goal_y = self.odom_y + random.uniform(self.upper, self.lower)
            goal_ok = check_pos(self.goal_x, self.goal_y)

        for i in range(5):
            name = "cardboard_box_" + str(i)

            x = 0
            y = 0
            box_ok = False
            while not box_ok:
                x = np.random.uniform(-6, 6)
                y = np.random.uniform(-6, 6)
                box_ok = check_pos(x, y)
                distance_to_robot = np.linalg.norm([x - self.odom_x, y - self.odom_y])
                distance_to_goal = np.linalg.norm([x - self.goal_x, y - self.goal_y])
                if distance_to_robot < 1.5 or distance_to_goal < 1.5:
                    box_ok = False
            box_state = ModelState()
            box_state.model_name = name
            box_state.pose.position.x = x
            box_state.pose.position.y = y
            box_state.pose.position.z = 0.0
            box_state.pose.orientation.x = 0.0
            box_state.pose.orientation.y = 0.0
            box_state.pose.orientation.z = 0.0
            box_state.pose.orientation.w = 1.0
            self.set_state.publish(box_state)
        

        # Generate initial goal position
        #self.goal_x, self.goal_y = self.generate_goal_position()

        # Gather initial state information
        # lidar_data = self.lidar.read_data()
        # odometry_data = self.robot.get_odometry()  # Example method
        # distance, angle = self.calculate_goal_distance_and_angle()

        while not self.unpause.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')

        try:
            self.unpause.call_async(Empty.Request())
        except:
            print("/gazebo/unpause_physics service call failed")

        time.sleep(TIME_DELTA)

        while not self.pause.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')

        try:
            self.pause.call_async(Empty.Request())
        except:
            print("/gazebo/pause_physics service call failed")

        v_state = []
        v_state[:] = lidar_data[:]
        laser_state = [v_state]

        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )

        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y

        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))

        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle

        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        robot_state = [distance, theta, 0.0, 0.0]
        state = np.append(laser_state, robot_state)
        return state


    def step(self, action):

        global lidar_data
        target = False

        # Publish the robot action
        vel_cmd = Twist()
        vel_cmd.linear.x = float(action[0])
        vel_cmd.angular.z = float(action[1])
        self.vel_pub.publish(vel_cmd)

        # Apply action to the robot
        self.robot.apply_action(action)
        self.step_count += 1

        # Simulate environment for a timestep (replace with your simulation logic)
        #self.simulate_step()

        while not self.unpause.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        try:
            self.unpause.call_async(Empty.Request())
        except:
            print("/unpause_physics service call failed")

        # propagate state for TIME_DELTA seconds
        time.sleep(TIME_DELTA)

        while not self.pause.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        try:
            pass
            self.pause.call_async(Empty.Request())
        except (rclpy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")

        # Gather state information
        # lidar_data = self.lidar.read_data()
        # odometry_data = self.robot.get_odometry()
        # distance, angle = self.calculate_goal_distance_and_angle()
        # collision = self.check_collision(lidar_data)
             # read lidar laser state
        done, collision, min_laser = self.check_collision_collision(lidar_data)
        v_state = []
        v_state[:] = lidar_data[:]
        laser_state = [v_state]

        # Calculate robot heading from odometry data
        self.odom_x = last_odom.pose.pose.position.x
        self.odom_y = last_odom.pose.pose.position.y
        quaternion = Quaternion(
            last_odom.pose.pose.orientation.w,
            last_odom.pose.pose.orientation.x,
            last_odom.pose.pose.orientation.y,
            last_odom.pose.pose.orientation.z,
        )
        euler = quaternion.to_euler(degrees=False)
        angle = round(euler[2], 4)

        # Calculate distance to the goal from the robot
        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )

        # Calculate the relative angle between the robots heading and heading toward the goal
        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y
        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))
        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle
        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        # Calculate reward
        reward = -1  # Default reward for each timestep
        if collision:
            reward = -100
        elif distance < self.goal_reached_distance:
            reward = 100

        # Handle termination conditions
        done = True if collision or distance < GOAL_REACHED_DIST or self.step_count >= self.max_episode_steps else False

        # Return state, reward, done, and info
        robot_state = [distance, theta, action[0], action[1]]
        state = np.append(laser_state, robot_state)
        return state, reward, done, target

    # Helper functions (replace with your implementations)
    # def simulate_step(self):
    #     # Advance the environment's state by one timestep
    #     pass

    # def calculate_goal_distance_and_angle(self):
    #     # Calculate distance and angle to the goal
    #     pass

    def check_collision(self, laser_data):
        # Check for collisions using lidar data
        min_laser = min(laser_data)
        if min_laser < COLLISION_DIST:
            return True, True, min_laser
        return False, False, min_laser

    def generate_goal_position(self):
        # Generate a new random goal position within the environment
        pass

class Odom_subscriber(Node):

    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription

    def odom_callback(self, od_data):
        global last_odom
        last_odom = od_data

class Lidar_subscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/drlbot/laser/scan',
            self.lidar_callback,
            10  # Adjust the queue size as needed
        )
        self.subscription

        self.gaps = [[-np.pi / 2 - 0.03, -np.pi / 2 + np.pi / environment_dim]]
        for m in range(environment_dim - 1):
            self.gaps.append(
                [self.gaps[m][1], self.gaps[m][1] + np.pi / environment_dim]
            )
        self.gaps[-1][-1] += 0.03

    def lidar_callback(self, v):
        global lidar_data
        data = v.ranges[90:270]
        data[~np.isfinite(np.array(data))] = 10
        data=np.array(data)
        lidar_data = data[::9]

def check_pos(x, y):
    goal_ok = True

    if -3.8 > x > -6.2 and 6.2 > y > 3.8:
        goal_ok = False

    if -1.3 > x > -2.7 and 4.7 > y > -0.2:
        goal_ok = False

    if -0.3 > x > -4.2 and 2.7 > y > 1.3:
        goal_ok = False

    if -0.8 > x > -4.2 and -2.3 > y > -4.2:
        goal_ok = False

    if -1.3 > x > -3.7 and -0.8 > y > -2.7:
        goal_ok = False

    if 4.2 > x > 0.8 and -1.8 > y > -3.2:
        goal_ok = False

    if 4 > x > 2.5 and 0.7 > y > -3.2:
        goal_ok = False

    if 6.2 > x > 3.8 and -3.3 > y > -4.2:
        goal_ok = False

    if 4.2 > x > 1.3 and 3.7 > y > 1.5:
        goal_ok = False

    if -3.0 > x > -7.2 and 0.5 > y > -1.5:
        goal_ok = False

    if x > 4.5 or x < -4.5 or y > 4.5 or y < -4.5:
        goal_ok = False

    return goal_ok
