import gym
import argparse

from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines import PPO2
from gym.wrappers.time_limit import TimeLimit

import gym_dyn_reacher
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
args = parser.parse_args()


def create_env():
    env = gym.make(
        'DynReacherForce-v0',
        frequency=args.freq,
        delay=args.delay,
        distance_reduction_reward_weight=args.dist_rew,
        electricity_reward_weight=args.elec_rew,
        stuck_joint_reward_weight=args.stuck_rew,
    )
    return env


env = SubprocVecEnv([create_env])

model = PPO2(MlpPolicy, env, verbose=1, tensorboard_log="tensorboard", n_steps=args.n_steps, full_tensorboard_log=True)

tb_args = {
    "n_steps": args.n_steps,
    "freq": args.freq,
    "delay": args.delay,
    "dist_rew": args.dist_rew,
    "elec_rew": args.elec_rew,
    "stuck_rew": args.stuck_rew,
}
log_name = "PPO2," + ",".join(['{} {}'.format(key, val) for key, val in tb_args.items()])

if not args.play:
    total_timesteps = int(round((args.wall_hours * 60 * 60) * args.freq))
    print('timesteps needed:', total_timesteps)
    model.learn(total_timesteps=total_timesteps, tb_log_name=log_name)
    model.save("ppo2_cartpole")
else:
    # env = SubprocVecEnv([lambda: gym.make('DynReacherForce-v0')])
    model = PPO2.load("ppo2_cartpole", env=env)

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
