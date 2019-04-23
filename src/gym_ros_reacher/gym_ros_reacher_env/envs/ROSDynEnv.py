import gym
from gym import utils, spaces
import numpy as np
from math import sin, cos, radians, pi
import time
import rospy
from std_msgs.msg import Float32MultiArray
from dynamixel_driver.srv import SetDynamixelPositions as DynamixelPositions

##### Dynamixel safety setup
_lim_safety = radians(30)
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
    return np.array([y,z])

##### GYM STUFF
class ROSDynEnv(gym.Env):
    def __init__(self):
        # Ros stuff
        rospy.init_node("RosDynGym")

        # Subscribe
        self.Ros_pos = rospy.Subscriber("dynamixel_present_position", Float32MultiArray, self.get_pos)
        self.Ros_vel = rospy.Subscriber("dynamixel_present_velocity", Float32MultiArray, self.get_vel)
        self.Ros_trq = rospy.Subscriber("dynamixel_present_torque", Float32MultiArray, self.get_trq)

        # service
        rospy.wait_for_service('dynamixel_set_positions_service')
        self.moveRobot = rospy.ServiceProxy('dynamixel_set_positions_service', DynamixelPositions)

        # Observations
        self.pos    = [0.0, 0.0]
        self.vel    = [0.0, 0.0]
        self.trq    = [0.0, 0.0]
        self.target = [0.0, 0.0]
        self.dist2target = 0.0
        time.sleep(1)

        # Gym stuff
        # utils.EzPickle.__init__(self)
        self.action_space = spaces.Box(low=motor_lim_angle_lo , high=motor_lim_angle_hi, dtype=motor_lim_angle_hi.dtype)
        _obs = self.get_obs()
        high = np.inf*np.ones(_obs.size)
        low = -high
        self.observation_space = spaces.Box(low, high, dtype=np.float32)

    def get_pos(self,data):
        self.pos = np.array(data.data, dtype=np.double)
        # print('get_pos: ', self.pos)

    def get_vel(self,data):
        self.vel = data.data
        # print('get_vel: ', data.data)

    def get_trq(self,data):
        self.trq = data.data
        # print('get_trq: ', data)

    # Env-gym functions
    def step(self, action):
        obs = self.get_obs()
        reward = self.get_reward()
        self.moveRobot(action)
        done = False
        print("
              "

        )

        return obs, reward, done, {}

    def reset(self):
        start_joint_angles = _rand_joint_angles()
        self.target = _forward(_rand_joint_angles())
        random_move = _rand_joint_angles()
        self.moveRobot(random_move)
        # return self.get_obs()

    def get_reward(self):
        return -self.dist2target

    def get_obs(self):
        self.dist2target = np.linalg.norm(self.pos - self.target)
        return np.concatenate([
            self.target,
            self.pos,
            self.trq,
            [self.dist2target]
        ])
