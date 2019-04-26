import gym
import gym_dyn_reacher
from gym_dyn_reacher.envs.DynReacherVelChangeEnv import _forward, _rand_joint_angles
import numpy as np

env = gym.make("DynReacherForce-v0")

env.reset()

env.step([3, 3])
print(env.env.sim.data.ctrl)

i = 0
while True:
    env.render()
    env.step(env.action_space.sample())  # take a random action
    i += 1
    if i > 500:
        env.reset()
        i = 0
