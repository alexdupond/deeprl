import gym
from gym import utils, spaces
import numpy as np
from math import sin, cos, radians, pi
import time
import rospy
from std_msgs.msg import Float32MultiArray
from dynamixel_driver.srv import SetDynamixelPositions as DynamixelPositions
import time

from threading import Lock, Event

##### Dynamixel safety setup
#  sudo chmod a+rw /dev/t
_joint_lim_safety = radians(40)
joint_lim_lo = np.array([-1.1, -1.8], dtype=np.float32) + _joint_lim_safety
joint_lim_hi = np.array([1.75, 1.8], dtype=np.float32) - _joint_lim_safety

_joint_penalty_lim = radians(10)
joint_penalty_lim_lo = joint_lim_lo + _joint_penalty_lim
joint_penalty_lim_hi = joint_lim_hi - _joint_penalty_lim

trq_lim_lo = np.array([-1, -1], dtype=np.float32)
trq_lim_hi = np.array([1, 1], dtype=np.float32)


def _rand_joint_angles():
    return np.random.uniform(joint_penalty_lim_lo, joint_penalty_lim_hi)


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
    return np.array([y, z])


was_initted = False


class ROSDynForceEnv(gym.Env):
    def __init__(self, distance_reduction_reward_weight: float, electricity_reward_weight: float,
                 stuck_joint_reward_weight: float):
        global was_initted
        assert not was_initted
        was_initted = True
        print("NICE! ")
        # Ros stuff #######################################################################################
        rospy.init_node("RosDynGym")
        self.lock = Lock()
        self.input_received = Event()

        # Observations
        self.pos = np.zeros(2)
        self.pos_updated = False
        self.vel = np.zeros(2)
        self.vel_updated = False
        self.trq = np.zeros(2)
        self.trq_updated = False
        self.target = np.zeros(2)
        self.dist2target = 0.0
        self.last_potential = 0

        # reward weights
        self.distance_reduction_reward_weight = distance_reduction_reward_weight
        self.electricity_reward_weight = electricity_reward_weight
        self.stuck_joint_reward_weight = stuck_joint_reward_weight

        # Subscribe
        self.Ros_pos = rospy.Subscriber("dynamixel_present_position", Float32MultiArray, self.get_pos_cb)
        self.Ros_vel = rospy.Subscriber("dynamixel_present_velocity", Float32MultiArray, self.get_vel_cb)
        self.Ros_trq = rospy.Subscriber("dynamixel_present_torque", Float32MultiArray, self.get_trq_cb)

        # Publishers
        self.set_trq = rospy.Publisher("dynamixel_set_torque", Float32MultiArray, queue_size=10)

        # service
        rospy.wait_for_service('dynamixel_set_positions_service')
        self.moveRobot = rospy.ServiceProxy('dynamixel_set_positions_service', DynamixelPositions)

        # Gym stuff
        self.action_space = spaces.Box(low=trq_lim_lo, high=trq_lim_hi, dtype=np.float32)
        _obs = self.get_obs()
        high = np.inf * np.ones(_obs.size)
        low = -high
        self.observation_space = spaces.Box(low, high, dtype=np.float32)

        self.reset()

    def get_pos_cb(self, data):
        self.pos = np.array(data.data)
        self.pos_updated = True
        self.attempt_release()

    def get_vel_cb(self, data):
        self.vel = np.array(data.data)
        self.vel_updated = True
        self.attempt_release()

    def get_trq_cb(self, data):
        self.trq = np.array(data.data)
        self.trq_updated = True
        self.attempt_release()

    def attempt_release(self):
        with self.lock:
            should_release = self.pos_updated and self.vel_updated and self.trq_updated
            if should_release:
                self.pos_updated = self.vel_updated = self.trq_updated = False
        if should_release:
            self.input_received.set()

    def wait_for_input(self):
        self.input_received.wait()
        self.wait_reset()

    def wait_reset(self):
        self.input_received.clear()

    def step(self, action):
        last_distance = self._get_distance()
        action = np.clip(action, trq_lim_lo, trq_lim_hi)  # Clip to action space
        self.set_trq.publish(data=action)
        self.wait_reset()
        self.wait_for_input()
        reward = self.get_reward(last_distance, self._get_distance())
        obs = self.get_obs()
        done = np.any(obs[:2] < joint_lim_lo) or np.any(obs[:2] > joint_lim_hi)
        return obs, reward, done, {}

    def reset(self):
        start_joint_angles = _rand_joint_angles()
        self.target = _forward(_rand_joint_angles())
        self.moveRobot(start_joint_angles)
        self.wait_reset()
        self.wait_for_input()
        return self.get_obs()

    def _get_distance(self):
        vec = _forward(self.get_pos()) - self.target
        return np.linalg.norm(vec)

    def get_reward(self, old_distance, new_distance):
        theta = self.get_pos()
        omega = self.get_vel()
        torque = self.get_trq()
        electricity = (
                np.sum(np.abs(torque * omega))  # work torque*angular_velocity
                + 0.1 * np.sum(np.abs(torque))  # stall torque require some energy
        )
        stuck_joints = np.sum(
            np.less(theta, joint_penalty_lim_lo) + np.greater(theta, joint_penalty_lim_hi))
        distance_reduction = - (new_distance - old_distance)
        return (
                + distance_reduction * self.distance_reduction_reward_weight
                + electricity * self.electricity_reward_weight
                + stuck_joints * self.stuck_joint_reward_weight
        )

    def get_pos(self):
        with self.lock:
            return self.pos

    def get_vel(self):
        with self.lock:
            return self.vel

    def get_trq(self):
        with self.lock:
            return self.trq

    def get_obs(self):
        pos = self.get_pos()
        obs = np.concatenate([
            pos,
            self.target,
            self.get_vel(),
            np.cos(pos),
            np.sin(pos),
            (_forward(pos) - self.target)
        ])
        return obs
