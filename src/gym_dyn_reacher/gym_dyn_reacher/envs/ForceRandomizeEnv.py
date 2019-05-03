import os
import numpy as np
from gym import utils, spaces
from gym.envs.mujoco import mujoco_env
from math import sin, cos, radians, pi
from mujoco_py.generated import const

_lim_safety = radians(10)

motor_lim_angle_lo = np.array([-1., -2.]) + _lim_safety
motor_lim_angle_hi = np.array([1.8, 2.]) - _lim_safety


def _rand_joint_angles():
    return np.random.uniform(motor_lim_angle_lo, motor_lim_angle_hi)


def _forward(j):
    """forward kinematics"""
    [j0, j1] = j
    y = z = 0
    # motor height
    m_z = 0.0465
    mjoint_to_tip_l = 0.035
    # red angle body
    a_phi = -0.2363
    a_l = .07
    # j0 position
    z += mjoint_to_tip_l + 0.008  # 8 mm off the ground
    # j1 position
    y += -sin(j0 + a_phi) * a_l
    z += cos(j0 + a_phi) * a_l
    # tip position
    j1_offset = (pi / 2 + a_phi) * 2 - pi
    y += -sin(j0 + j1_offset + j1) * mjoint_to_tip_l
    z += cos(j0 + j1_offset + j1) * mjoint_to_tip_l
    return y, z


class DynReacherVelChangeEnv(mujoco_env.MujocoEnv, utils.EzPickle):
    def __init__(self):
        utils.EzPickle.__init__(self)
        model_path = os.path.join(os.path.dirname(__file__), "../assets", "reacher.xml")
        self.target_pos = np.array([0, 0])
        mujoco_env.MujocoEnv.__init__(self, model_path, 2)
        aspace = self.action_space
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=aspace.dtype)
        self.reset_model()


    def step(self, a):
        ctrl = np.concatenate((a[:2], self.target_pos))
        reward = self._get_reward()
        self.do_simulation(ctrl, self.frame_skip)
        ob = self._get_obs()
        done = False
        return ob, reward, done, {}

    def viewer_setup(self):
        self.viewer.cam.trackbodyid = 0
        self.viewer.cam.elevation = -10
        self.viewer.cam.azimuth = 180
        self.viewer.cam.lookat[:3] = [0, 0, 0.07]

    def reset_model(self):
        start_joint_angles = _rand_joint_angles()
        self.target_pos = _forward(_rand_joint_angles())
        qpos = np.concatenate((start_joint_angles, self.target_pos))
        qvel = np.concatenate((np.random.uniform(-1, 1, 2), [0, 0]))
        self.set_state(qpos, qvel)
        return self._get_obs()

    def _get_reward(self):
        vec = self.get_body_com("tip") - self.get_body_com("target")
        return - np.linalg.norm(vec)

    def _get_obs(self):
        theta = self.sim.data.qpos.flat[:2]
        return np.concatenate([
            np.cos(theta),
            np.sin(theta),
            self.sim.data.qpos.flat,
            self.sim.data.qvel.flat[:2],
            (self.get_body_com("tip") - self.get_body_com("target"))[1:],
        ])



