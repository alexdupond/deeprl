#!/usr/bin/env python

import random
import rospy
from two_joint_env.srv import Step as StepSrv
import time


def main():
    step = rospy.ServiceProxy('step', StepSrv)
    rospy.wait_for_service('step')

    count = 0
    start_time = time.time()
    while time.time() - start_time < 10:
        step.call(actions=[random.random() - 0.5, random.random() - 0.5])
        count += 1
    print(count / 10, 'fps')


if __name__ == '__main__':
    main()