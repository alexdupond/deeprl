#!/usr/bin/env python3
import numpy as np
from math import sin, cos, radians, pi
import time
import rospy
from std_msgs.msg import Float32MultiArray
from dynamixel_driver.srv import SetDynamixelPositions as DynamixelPositions


_lim_safety = radians(30)

motor_lim_angle_lo = np.array([-1., -1.8]) #+ _lim_safety
motor_lim_angle_hi = np.array([1.8, 1.8]) #- _lim_safety

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

class ROSDynReacherVelChangeEnv():
    def __init__(self):

        ##Ros stuff
        rospy.init_node("RosDynGym")

        #Subscribe
        self.Ros_pos = rospy.Subscriber("dynamixel_present_position", Float32MultiArray, self.get_pos)
        self.Ros_vel = rospy.Subscriber("dynamixel_present_velocity", Float32MultiArray, self.get_vel)
        self.Ros_trq = rospy.Subscriber("dynamixel_present_torque"  , Float32MultiArray, self.get_trq)

        #service
        rospy.wait_for_service('dynamixel_set_positions_service')

    #### Ros Callback
    def get_pos(self,data):
        print('pos: ', data.data)

    def get_vel(self,data):
        print('vel: ', data.data)

    def get_trq(self,data):
        print('trq: ', data.data)


print("Launching ros_test.py")
run = ROSDynReacherVelChangeEnv()

time.sleep(1.0)
moveRobot = rospy.ServiceProxy('dynamixel_set_positions_service', DynamixelPositions)

moveto1 = [1,1]
moveto2 = [0,0]
moveR = _rand_joint_angles()
moveRobot( moveR)

print("\n Random movement was: ",moveR,"\n")

time.sleep(1)
moveRobot(moveto2)
time.sleep(1)

print("Shutting down")
# try:
#     rospy.spin()
# except KeyboardInterrupt:
#     print("Shutting down")
