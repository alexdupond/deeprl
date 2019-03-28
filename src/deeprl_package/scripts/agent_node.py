#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

def callback(msg):
    pos = msg.data 
    rospy.loginfo(pos)

def main():
    rospy.init_node('agent')
    # Syntac ("topic name", msgs class/type, callback-function)
    rospy.Subscriber("/MotorPositions", Float32MultiArray, callback)
    rospy.spin()

def setMotor():
    pub = rospy.Publisher('/MotorPositions')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg_to_motor = Float32MultiArray()
        msg_to_motor.data = 5
        rospy.loginfo(msg_to_motor)
        pub.publish(msg_to_motor)
        rate.sleep()

if __name__ == '__main__':
    try:
        setMotor()
    except rospy.ROSInterruptException: 
        pass
 