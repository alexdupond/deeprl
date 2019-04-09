import gym
import gym_dyn_reacher
from gym_dyn_reacher.envs.DynReacherVelChangeEnv import _forward, _rand_joint_angles
import numpy as np
import time

env = gym.make("DynReacherVelChange-v0")


for _ in range(10):
    angles = _rand_joint_angles()
    tip_pos = _forward(angles)
    env.env.set_state(np.concatenate((angles, tip_pos)), np.zeros(4))

    target_pos = env.env.get_body_com("target")
    _tip_pos = env.env.get_body_com("tip")

    err = abs(np.linalg.norm(target_pos - _tip_pos))
    env.render()
    time.sleep(1)

    assert err < 1e-3, err

print('test succeeded')
