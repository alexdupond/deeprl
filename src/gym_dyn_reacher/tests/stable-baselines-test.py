import gym
import argparse
import os

from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines import PPO2
from gym.wrappers.time_limit import TimeLimit

import gym_dyn_reacher
import gym_ros_reacher
from gym_dyn_reacher.envs.DynReacherForceEnv import DynReacherForceEnv

parser = argparse.ArgumentParser()
parser.add_argument("--play", action='store_const', const=True, default=False)
parser.add_argument("--n_steps", type=int, required=True)
parser.add_argument("--freq", type=int, required=True)
parser.add_argument("--delay", type=float, required=True)
parser.add_argument("--dist_rew", type=float, required=True)
parser.add_argument("--elec_rew", type=float, required=True)
parser.add_argument("--stuck_rew", type=float, required=True)
parser.add_argument("--wall_hours", type=float, required=True)
parser.add_argument("--exit_reward", type=float, required=True)
parser.add_argument("--real", type=str, required=True)
parser.add_argument("--model_path")
args = parser.parse_args()

assert args.real in ['true', 'false'], 'real must be true or false'
args.real = args.real == 'true'

tb_args = {
    "real": args.real,
    "n_steps": args.n_steps,
    "freq": args.freq,
    "delay": args.delay,
    "dist_rew": args.dist_rew,
    "elec_rew": args.elec_rew,
    "stuck_rew": args.stuck_rew,
    "exit_reward": args.exit_reward,
}

train_name = "PPO2," + ",".join(['{} {}'.format(key, val) for key, val in tb_args.items()])

assert os.path.isdir("models"), 'models should be a folder'
if args.model_path is None:
    args.model_path = "models/" + train_name
assert os.path.exists(args.model_path) == args.play, 'model does not exist' if args.play else 'model already exists'


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

model = PPO2(MlpPolicy, env, verbose=1, tensorboard_log="tensorboard", n_steps=args.n_steps, full_tensorboard_log=True)

if not args.play:
    total_timesteps = int(round((args.wall_hours * 60 * 60) * args.freq))
    print('timesteps needed:', total_timesteps)
    model.learn(total_timesteps=total_timesteps, tb_log_name=train_name)
    model.save(args.model_path)
else:
    model = PPO2.load(args.model_path, env=env)

    _env = create_env()
    # Enjoy trained agent
    obs = _env.reset()

    while True:
        for _ in range(200):
            action, _states = model.predict([obs])
            obs, rewards, done, info = _env.step(action[0])
            _env.render()
            if done:
                break
        _env.reset()
