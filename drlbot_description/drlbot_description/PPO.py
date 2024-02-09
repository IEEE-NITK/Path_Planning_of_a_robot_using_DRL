import os
import time
import gym
import numpy as np
from stable_baselines3 import TD3
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.results_plotter import plot_results
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback
from replay_buffer import ReplayBuffer
from cust_env import ObstacleAvoidanceEnv

device = "cuda" if torch.cuda.is_available() else "cpu"
seed = 0
eval_freq = 5e3
max_ep = 500
eval_ep = 10
max_timesteps = 5e6
expl_noise = 1
expl_decay_steps = 500000
expl_min = 0.1
batch_size = 40
discount = 0.99999
tau = 0.005
policy_noise = 0.2
noise_clip = 0.5
policy_freq = 2
buffer_size = 1e6
file_name = "TD3_velodyne"
save_model = True
load_model = False
random_near_obstacle = True

environment_dim = 20
robot_dim = 4
env = ObstacleAvoidanceEnv(environment_dim)
env = DummyVecEnv([lambda: env])
model = TD3(
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
                and min(state[4:-8]) < 0.6
                and count_rand_actions < 1 ):
                count_rand_actions = np.random.randint(8, 15)
                random_action = np.random.uniform(-1, 1, 2)

            if count_rand_actions > 0:
                count_rand_actions -= 1
                action = random_action
                action[0] = -1

        obs, reward, done, info = env.step(a_in)

        model.replay_buffer.add(obs, action, reward, done, obs)
        episode_reward += reward
        episode_timesteps += 1
        timestep += 1
        timesteps_since_eval += 1

        model.learn(total_timesteps=timestep)

    if timesteps_since_eval >= eval_freq:
        mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=eval_ep)
        print(f"Evaluation mean reward: {mean_reward:.2f} +/- {std_reward:.2f}")
        timesteps_since_eval %= eval_freq

