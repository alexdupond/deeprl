import gym
import gym_dyn_reacher
from gym_dyn_reacher.envs.DynReacherVelChangeEnv import _forward, _rand_joint_angles
import numpy as np

env = gym.make("DynReacherVelChange-v0")

try:
    for _ in range(100):
        angles = _rand_joint_angles()
        tip_pos = _forward(angles)
        env.env.set_state(np.concatenate((angles, tip_pos)), np.zeros(4))

        target_pos = env.env.get_body_com("target")
        _tip_pos = env.env.get_body_com("tip")

        assert (abs(np.linalg.norm(target_pos - _tip_pos)) < 1e-3)
    print('test successful')
except AssertionError as error:
    print(error)
finally:
    env.close()
