import gym
import gym_dyn_reacher
from gym_dyn_reacher.envs.DynReacherVelChangeEnv import _forward, _rand_joint_angles
import numpy as np

env = gym.make("DynReacherForce-v0")

angles = np.array([0, 0])
tip_pos = _forward(angles)
env.env.set_state(np.concatenate((angles, tip_pos)), np.zeros(4))

print(tip_pos)
print(env.env._get_obs())

env.close()
