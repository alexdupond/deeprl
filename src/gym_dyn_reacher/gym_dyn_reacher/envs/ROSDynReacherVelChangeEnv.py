import numpy as np
import gym
from gym import utils, spaces
from math import sin, cos, radians, pi

import rospy
from std_msgs.msg import Float32MultiArray


# register(
#     id='ROSDynReacherVelChange-v0',
#     entry_point='gym_dyn_reacher.envs:ROSDynReacherVelChangeEnv',
#     max_episode_steps=500,
# )


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


class ROSDynReacherVelChangeEnv(gym.Env):
    def __init__(self):

        ##Ros stuff
        rospy.init_node("RosDynGym")

        #Subscribe
        self.Ros_pos = rospy.Subscriber("dynamixel_present_position", Float32MultiArray, self.get_pos())
        self.Ros_vel = rospy.Subscriber("dynamixel_present_velocity", Float32MultiArray, self.get_vel())
        self.Ros_trq = rospy.Subscriber("dynamixel_present_torque", Float32MultiArray, self.get_trq())

        #service
        rospy.wait_for_service('dynamixel_set_positions_service')

        ## Gym stuff
        self.target_pos = np.array([0, 0])
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=aspace.dtype)
        self.observation_space = None

    #### Ros Callback
    def get_pos(self,data):
        print('pos: ', data)
        return data

    def get_vel(self,data):
        print('vel: ', data)
        return data

    def get_trq(self,data):
        print('trq: ', data)
        return data


    ## Env-gym functions

    def step(self, a):


        ctrl = np.concatenate((a[:2], self.target_pos))
        reward = self._get_reward()
        self.do_simulation(ctrl, self.frame_skip)
        ob = self._get_obs()
        done = False
        return ob, reward, done, {}


    def reset_model(self):

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
