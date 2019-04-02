#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Int32, Float32MultiArray
import time

from two_joint_env.srv import Step as StepSrv

# rospy allocates each topic / service it's own thread,
# which means we can lock the service and unlock it with a topic


def main():
    rospy.init_node("two_joint_env")

    pubStartSim = rospy.Publisher('startSimulation', Bool, queue_size=10)
    pubStopSim = rospy.Publisher('stopSimulation', Bool, queue_size=10)
    pubEnableSyncMode = rospy.Publisher('enableSyncMode', Bool, queue_size=10)
    pubTriggerNextStep = rospy.Publisher('triggerNextStep', Bool, queue_size=10)
    pubMultiJointCommandSim = rospy.Publisher('multiJointCommandSim', Float32MultiArray, queue_size=10)

    def step(req):
        print(req)
        # set action in scene
        pubMultiJointCommandSim.publish(data=req.actions)
        # trigger next step
        pubTriggerNextStep.publish(True)
        # wait for step to be done
        stateMsg = rospy.wait_for_message('jointStateSim', Float32MultiArray)
        # retrieve new state
        # return new state
        return {'state': stateMsg.data}

    rospy.Service('step', StepSrv, step)

    rospy.wait_for_message('simulationState', Int32)

    time.sleep(1.0)
    pubEnableSyncMode.publish(True)
    time.sleep(0.5)
    pubStartSim.publish(True)

    rospy.spin()


if __name__ == '__main__':
    main()
