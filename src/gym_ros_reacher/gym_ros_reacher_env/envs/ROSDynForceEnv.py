import gym
from gym import utils, spaces
import numpy as np
from math import sin, cos, radians, pi
import time
import rospy
from std_msgs.msg import Float32MultiArray
from dynamixel_driver.srv import SetDynamixelPositions as DynamixelPositions

##### Dynamixel safety setup
#  sudo chmod a+rw /dev/t
_lim_safety = radians(10)

_s = radians(40)
motor_lim_angle_lo = np.array([-1.1, -1.8], dtype=np.float32) + _s
motor_lim_angle_hi = np.array([1.75, 1.8], dtype=np.float32) - _s

safety_motor_lim_angle_lo = motor_lim_angle_lo + _lim_safety
safety_motor_lim_angle_hi = motor_lim_angle_hi - _lim_safety

trq_lim_lo = np.array([-0.1, -0.1], dtype=np.float32)
trq_lim_hi = np.array([0.1, 0.1], dtype=np.float32)


def _rand_joint_angles():
    return np.random.uniform(safety_motor_lim_angle_lo, safety_motor_lim_angle_hi)


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
    j1_offset = (pi / 2 + a_phi) * 2 - p
    y += -sin(j0 + j1_offset + j1) * mjoint_to_tip_l
    z += cos(j0 + j1_offset + j1) * mjoint_to_tip_l
    return np.array([y,z])

was_initted = False
##### GYM STUFF
class ROSDynForceEnv(gym.Env):
    def __init__(self):
        global was_initted
        assert not was_initted
        was_initted = True
        print("NICE! ")
        # Ros stuff #######################################################################################
        rospy.init_node("RosDynGym")

        # Subscribe
        self.Ros_pos = rospy.Subscriber("dynamixel_present_position", Float32MultiArray, self.get_pos)
        self.Ros_vel = rospy.Subscriber("dynamixel_present_velocity", Float32MultiArray, self.get_vel)
        self.Ros_trq = rospy.Subscriber("dynamixel_present_torque", Float32MultiArray, self.get_trq)

        # Publishers
        self.set_trq = rospy.Publisher("dynamixel_set_torque", Float32MultiArray, queue_size=10)

        # service
        rospy.wait_for_service('dynamixel_set_positions_service')
        self.moveRobot = rospy.ServiceProxy('dynamixel_set_positions_service', DynamixelPositions)

        # Observations
        self.pos    = np.zeros(2)
        self.vel    = np.zeros(2)
        self.trq    = np.zeros(2)
        self.target = np.zeros(2)
        self.dist2target = 0.0
        time.sleep(1)

        # Gym stuff #######################################################################################
        self.action_space = spaces.Box(low=trq_lim_lo, high=trq_lim_hi, dtype=np.float32 )
        _obs = self.get_obs()
        high = np.inf*np.ones(_obs.size)
        low = -high
        self.observation_space = spaces.Box(low, high, dtype=np.float32)

        # Print cycle
        self.loopCount = 0.
        self.loopLimit = 10.
        self.sumReward = 0.
        self.nrSteps = 0

        self.reset()

    def get_pos(self,data):
        self.pos = np.array(data.data)
        # print('get_pos: ', self.pos)

    def get_vel(self,data):
        self.vel = np.array(data.data)
        # print('get_vel: ', data.data)

    def get_trq(self,data):
        self.trq = np.array(data.data)
        self.wait = False
        # print('get_trq: ', data)

    # Env-gym functions
    def step(self, action):

        old_potential = self._calc_potential()
        self.wait = True
        while self.wait:
            pass

        obs = self.get_obs()
        self.nrSteps += 1
        reward = self.get_reward(action, old_potential)
        done = False

        # Reset if pos out of scope
        low_pos_limit = self.pos > motor_lim_angle_lo
        high_pos_limit = self.pos < motor_lim_angle_hi
        if False in low_pos_limit:
            done = True
            #self.moveRobot(motor_lim_angle_lo + 0.1)
        if False in high_pos_limit:
            done = True
            #self.moveRobot(motor_lim_angle_hi - 0.1)

        if not done:
            action = np.clip(action, trq_lim_lo, trq_lim_hi) # Clip to action space
            self.set_trq.publish(data=action)

            # print cycle
            self.sumReward += reward
            if self.loopCount > self.loopLimit:
                print("##############  100 steps status ##################")
                print("# Action  : ", action)
                print("# Position: ", self.pos)
                print("# Target  : ", self.target)
                print("# Reward  : ", self.sumReward / self.loopCount)
                print("# Steps   : ", self.nrSteps)
                print("################################################\n")
                self.loopCount = 0
                self.sumReward = 0
            self.loopCount += 1

        return obs, reward, done, {}

    def reset(self):
        start_joint_angles = _rand_joint_angles()
        self.target = _forward(_rand_joint_angles())
        self.moveRobot(start_joint_angles)
        print("\n ...reseting robot")
        print("################################")
        print("# RESET ROBOT ")
        print("# Reset to: ", start_joint_angles )
        print("################################\n")
        # return self.get_obs()
        self.loopCount = 0
        self.sumReward = 0

    def _calc_potential(self):
        vec = _forward(self.pos) - self.target
        return - 200 * np.linalg.norm(vec)

    def get_reward(self, force, old_potential):
        theta = self.pos
        omega = self.vel
        #force = self.trq
        electricity_cost = (
                - 0.10 * np.sum(np.abs(force * omega))  # work torque*angular_velocity
                - 0.01 * np.sum(np.abs(force))  # stall torque require some energy
        )
        stuck_joint_cost = -0.1 * np.sum(np.less(theta, safety_motor_lim_angle_lo) + np.greater(theta, safety_motor_lim_angle_hi))
        new_potential = self._calc_potential()
        return new_potential - old_potential + electricity_cost + stuck_joint_cost

    def get_obs(self):
        return np.concatenate([
            self.pos,
            self.target,
            np.cos(self.pos),
            np.sin(self.pos),
            (_forward( self.pos) - self.target)
        ])
