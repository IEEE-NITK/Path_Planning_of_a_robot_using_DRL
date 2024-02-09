import os
import time
import gym
import numpy as np
from stable_baselines3 import TD3, DDPG
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.results_plotter import plot_results
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback
import random
import matplotlib.pyplot as plt
import threading
import math
from collections import deque
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from squaternion import Quaternion
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import torch
import torch.nn as nn
import torch.nn.functional as F
from cust_env import ObstacleAvoidanceEnv
from cust_env import Lidar_subscriber
from cust_env import Odom_subscriber
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

GOAL_REACHED_DIST = 0.3
COLLISION_DIST = 0.35
TIME_DELTA = 0.2
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # cuda or cpu


        
class ReplayBuffer(object):
    def __init__(self, buffer_size, random_seed=123):
        """
        The right side of the deque contains the most recent experiences
        """
        self.buffer_size = buffer_size
        self.count = 0
        self.buffer = deque()
        random.seed(random_seed)

    def add(self, s, a, r, t, s2):
        experience = (s, a, r, t, s2)
        if self.count < self.buffer_size:
            self.buffer.append(experience)
            self.count += 1
        else:
            self.buffer.popleft()
            self.buffer.append(experience)

    def size(self):
        return self.count

    def sample_batch(self, batch_size):
        batch = []

        if self.count < batch_size:
            batch = random.sample(self.buffer, self.count)
        else:
            batch = random.sample(self.buffer, batch_size)

        s_batch = np.array([_[0] for _ in batch])
        a_batch = np.array([_[1] for _ in batch])
        r_batch = np.array([_[2] for _ in batch]).reshape(-1, 1)
        t_batch = np.array([_[3] for _ in batch]).reshape(-1, 1)
        s2_batch = np.array([_[4] for _ in batch])

        return s_batch, a_batch, r_batch, t_batch, s2_batch

    def clear(self):
        self.buffer.clear()
        self.count = 0

if __name__ == '__main__':

    rclpy.init(args=None)

    seed = 0  # Random seed number
    eval_freq = 5e3  # After how many steps to perform the evaluation
    max_ep = 500  # maximum number of steps per episode
    eval_ep = 10  # number of episodes for evaluation
    max_timesteps = 5e6  # Maximum number of steps to perform
    expl_noise = 1  # Initial exploration noise starting value in range [expl_min ... 1]
    expl_decay_steps = (
        500000  # Number of steps over which the initial exploration noise will decay over
    )
    expl_min = 0.1  # Exploration noise after the decay in range [0...expl_noise]
    batch_size = 40  # Size of the mini-batch
    discount = 0.99999  # Discount factor to calculate the discounted future reward (should be close to 1)
    tau = 0.005  # Soft target update variable (should be close to 0)
    policy_noise = 0.2  # Added noise for exploration
    noise_clip = 0.5  # Maximum clamping values of the noise
    policy_freq = 2  # Frequency of Actor network updates
    buffer_size = 1e6  # Maximum size of the buffer
    file_name = "td3_lidar"  # name of the file to store the policy
    save_model = True  # Weather to save the model or not
    load_model = False  # Weather to load a stored model
    random_near_obstacle = True  # To take random actions near obstacles or not

     # Create the training environment
    environment_dim = 20
    robot_dim = 4

    torch.manual_seed(seed)
    np.random.seed(seed)
    state_dim = environment_dim + robot_dim
    action_dim = 2
    max_action = 1

    env = ObstacleAvoidanceEnv()
    odom_subscriber = Odom_subscriber()
    lidar_subscriber = Lidar_subscriber()
    env = DummyVecEnv([lambda: env])

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(odom_subscriber)
    executor.add_node(lidar_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    model = DDPG(
    "MlpPolicy",
    env,
    learning_rate=1e-3,
    buffer_size=buffer_size,
    batch_size=batch_size,
    tau=tau,
    policy_delay=policy_freq,
    action_noise=NormalActionNoise(mean=np.zeros(2), sigma=0.1 * np.ones(2)),
    tensorboard_log="./tensorboard/"
    )
    replay_buffer = ReplayBuffer(buffer_size, seed)

    if load_model:
        try:
            model = TD3.load(file_name, env=env)
        except:
            print("Could not load the stored model parameters.")

    eval_callback = CheckpointCallback(save_freq=int(eval_freq/max_ep), save_path="./models/", name_prefix=file_name)

    timestep = 0
    timesteps_since_eval = 0
    episode_num = 0

    while timestep < max_timesteps:
        obs = env.reset()
        done = False
        episode_reward = 0
        episode_timesteps = 0

        while not done:
            action, _states = model.predict(obs)
            a_in = [(action[0] + 1) / 2, action[1]]

            if random_near_obstacle:
                if (
                    np.random.uniform(0, 1) > 0.85
                    and min(obs[4:-8]) < 0.6
                    and count_rand_actions < 1 ):
                    count_rand_actions = np.random.randint(8, 15)
                    random_action = np.random.uniform(-1, 1, 2)

                if count_rand_actions > 0:
                    count_rand_actions -= 1
                    action = random_action
                    action[0] = -1

            obs, reward, done, info = env.step(a_in)

            replay_buffer.add(obs, action, reward, done, obs)
            episode_reward += reward
            episode_timesteps += 1
            timestep += 1
            timesteps_since_eval += 1
            done_bool = 0 if episode_timesteps == max_ep else int(done)
            done = 1 if episode_timesteps == max_ep else int(done)

            model.learn(total_timesteps=timestep)

        if timesteps_since_eval >= eval_freq:
            mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=eval_ep)
            print(f"Evaluation mean reward: {mean_reward:.2f} +/- {std_reward:.2f}")
            timesteps_since_eval %= eval_freq

    rclpy.shutdown()
