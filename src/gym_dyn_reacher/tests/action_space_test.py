import gym
import gym_dyn_reacher
from gym_dyn_reacher.envs.DynReacherVelChangeEnv import _forward, _rand_joint_angles
import numpy as np

env = gym.make("DynReacherVelChange-v0")

env.reset()

env.step([3, 3])
print(env.env.sim.data.ctrl)

for _ in range(1000):
    env.render()
    env.step(env.action_space.sample())  # take a random action
env.close()
