import os
import numpy as np
from gym import utils, spaces
from gym.envs.mujoco import mujoco_env
from math import sin, cos, radians, pi
import queue

_joint_lim_safety = radians(40)
joint_lim_lo = np.array([-1.1, -1.8]) + _joint_lim_safety
joint_lim_hi = np.array([1.75, 1.8]) - _joint_lim_safety

_joint_penalty_lim = radians(10)
joint_penalty_lim_lo = joint_lim_lo + _joint_penalty_lim
joint_penalty_hi = joint_lim_hi - _joint_penalty_lim

trq_lim_lo = np.array([-1, -1], dtype=np.float32)
trq_lim_hi = np.array([1, 1], dtype=np.float32)


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


class DynReacherForceEnv(mujoco_env.MujocoEnv, utils.EzPickle):
    def __init__(self, frequency: int, delay: float, distance_reduction_reward_weight: float,
                 electricity_reward_weight: float, stuck_joint_reward_weight: float, exit_reward: float):
        assert frequency in (100, 50, 25, 10), 'frequency must be either  (100, 50, 25, 10)'
        frame_skip = int(round(100 / frequency))
        assert delay >= 0 and (delay * 100) % 1 < 1e-4, 'delay can only be in multiples of 0.01 s'
        self.ctrl_delay_steps = int(round(delay * 100))
        self.ctrl_queue = queue.Queue()
        self.target_pos = np.array([0, 0])
        self.latest_torque = np.array([0, 0])
        self.distance_reduction_reward_weight = distance_reduction_reward_weight
        self.electricity_reward_weight = electricity_reward_weight
        self.stuck_joint_reward_weight = stuck_joint_reward_weight
        self.exit_reward = exit_reward
        self.random = np.random.RandomState()

        utils.EzPickle.__init__(self)
        model_path = os.path.join(os.path.dirname(__file__), "../assets", "reacher-force.xml")
        mujoco_env.MujocoEnv.__init__(self, model_path, frame_skip)

        self.action_space = spaces.Box(low=trq_lim_lo, high=trq_lim_hi, dtype=self.action_space.dtype)
        self.reset_model()

    def reset_model(self, angles=None):
        start_joint_angles = angles if angles is not None else self._rand_joint_angles()
        self.target_pos = _forward(self._rand_joint_angles())
        qpos = np.concatenate((start_joint_angles, self.target_pos))
        qvel = np.zeros(4)
        self.set_state(qpos, qvel)
        self.latest_torque = np.array([0, 0])
        self.ctrl_queue = queue.Queue()
        for _ in range(self.ctrl_delay_steps):
            self.ctrl_queue.put([0, 0])
        return self._get_obs()

    def do_simulation(self, ctrl, n_frames):
        for _ in range(n_frames):
            self.ctrl_queue.put(ctrl)
        for _ in range(n_frames):
            ctrl = self.ctrl_queue.get()
            self.sim.data.ctrl[:] = np.concatenate((ctrl, self.target_pos))
            self.sim.step()

    def step(self, a):
        a = np.clip(a[:2], trq_lim_lo, trq_lim_hi)
        a = a * 0.1
        last_distance = self._get_distance()
        self.do_simulation(a[:2], self.frame_skip)
        new_distance = self._get_distance()
        reward = self._get_reward(last_distance, new_distance)
        obs = self._get_obs()
        theta = obs[:2]
        done = np.any(theta < joint_lim_lo) or np.any(theta > joint_lim_hi)
        if done:
            reward += self.exit_reward
        return obs, reward, done, {}

    def viewer_setup(self):
        self.viewer.cam.trackbodyid = 0
        self.viewer.cam.elevation = -10
        self.viewer.cam.azimuth = 180
        self.viewer.cam.lookat[:3] = [0, 0, 0.07]

    def _get_distance(self):
        vec = self.get_body_com("tip") - self.get_body_com("target")
        return np.linalg.norm(vec)

    def _get_reward(self, old_distance, new_distance):
        theta = self.sim.data.qpos.flat[:2]
        omega = self.sim.data.qvel.flat[:2]
        electricity = (
                np.sum(np.abs(self.latest_torque * omega))  # work torque*angular_velocity
                + 0.1 * np.sum(np.abs(self.latest_torque))  # stall torque require some energy
        )
        stuck_joints = np.sum(
            np.less(theta, joint_penalty_lim_lo) + np.greater(theta, joint_penalty_hi))
        distance_reduction = - (new_distance - old_distance)
        return (
                + distance_reduction * self.distance_reduction_reward_weight
                + electricity * self.electricity_reward_weight
                + stuck_joints * self.stuck_joint_reward_weight
        )

    def _get_obs(self):
        theta = self.sim.data.qpos.flat[:2]
        return np.concatenate([
            self.sim.data.qpos.flat,  # current pos, target pos
            self.sim.data.qvel.flat[:2],  # current vel
            np.cos(theta),
            np.sin(theta),
            (self.get_body_com("tip") - self.get_body_com("target"))[1:],
        ])

    def _rand_joint_angles(self):
        return self.random.uniform(joint_penalty_lim_lo, joint_penalty_hi)

    def seed(self, seed=None):
        self.random.seed(seed)
