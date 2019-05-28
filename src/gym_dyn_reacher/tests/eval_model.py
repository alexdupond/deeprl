import gym
import argparse
import os

import numpy as np
from progressbar import ProgressBar, Percentage
from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines import PPO2

import gym_dyn_reacher
import gym_ros_reacher

parser = argparse.ArgumentParser()
parser.add_argument("--real", type=str, required=True)
parser.add_argument("--freq", type=int, required=True)
parser.add_argument("--delay", type=float, required=True)
parser.add_argument("--dist_rew", type=float, required=True)
parser.add_argument("--elec_rew", type=float, required=True)
parser.add_argument("--stuck_rew", type=float, required=True)
parser.add_argument("--exit_reward", type=float, required=True)
parser.add_argument("--model_path", required=True)
args = parser.parse_args()

assert args.real in ['true', 'false']
args.real = args.real == 'true'

tb_args = {
    "real": args.real,
    "freq": args.freq,
    "delay": args.delay,
    "dist_rew": args.dist_rew,
    "elec_rew": args.elec_rew,
    "stuck_rew": args.stuck_rew,
    "exit_reward": args.exit_reward,
}

assert os.path.exists(args.model_path), 'cant find model'


def create_env():
    reward_args = {
        "distance_reduction_reward_weight": args.dist_rew,
        "electricity_reward_weight": args.elec_rew,
        "stuck_joint_reward_weight": args.stuck_rew,
        "exit_reward": args.exit_reward,
    }
    if args.real:
        env = gym.make('ROSForce-v0', **reward_args)
    else:
        env = gym.make('DynReacherForce-v0', **reward_args, frequency=args.freq, delay=args.delay)
    return env


env = SubprocVecEnv([create_env])

model = PPO2.load(args.model_path, env=env)

_env = create_env()

_env.seed(0)
obs = _env.reset()

episode_count = 100
episode_rewards = []
pbar = ProgressBar(widgets=[Percentage()], maxval=episode_count).start()
for episode_i in range(episode_count):
    pbar.update(episode_i)
    episode_reward = 0
    for _ in range(200):
        action, _states = model.predict([obs])
        obs, reward, done, info = _env.step(action[0])
        episode_reward += reward
        # _env.render()
        if done:
            break
    episode_rewards.append(episode_reward)
    obs = _env.reset()
pbar.finish()

mu_ep_rew = np.mean(episode_rewards)
s_sqr_ep_rew = np.var(episode_rewards, ddof=1)

mu_s_sqr = s_sqr_ep_rew / episode_count
mu_s = np.sqrt(mu_s_sqr)

print('mu episode reward:', mu_ep_rew, 'std', np.sqrt(s_sqr_ep_rew))
print('mu episode reward std: ', mu_s)
