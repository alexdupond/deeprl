function setMotorPositions_cb(msg)
    data = msg.data
    sim.setJointTargetVelocity(J0,data[1])
    sim.setJointTargetVelocity(J1,data[2])
end

function sysCall_init()
    robotHandle=sim.getObjectAssociatedWithScript(sim.handle_self)

    J0 = sim.getObjectHandle("J0")
    J1 = sim.getObjectHandle("J1")
    joints = {J0, J1}

    -- Check if the required ROS plugin is loaded
    moduleName=0
    moduleVersion=0
    index=0
    pluginNotFound=true
    while moduleName do
        moduleName,moduleVersion=sim.getModuleName(index)
        if (moduleName=='RosInterface') then
            pluginNotFound=false
        end
        index=index+1
    end
    if (pluginNotFound) then
        sim.displayDialog('Error','The RosInterface was not found.',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
    else
    -- If found then start the subscribers and publishers
        local MotorTopicName='multiJointCommandSim'
        local simulationTimeTopicName='simulationTime'
        local startSimulationName='startSimulation'
        local pauseSimulationName='pauseSimulation'
        local stopSimulationName='stopSimulation'
        local enableSyncModeName='enableSyncMode'
        local triggerNextStepName='triggerNextStep'
        local simulationStepDoneName='simulationStepDone'
        local simulationStateName='simulationState'
        local terminateControllerName='terminateController'

        local jointPositionsName='jointPositionsSim'
        local jointTorquesName='jointTorquesSim'
        local jointVelocitiesName='jointVelocitiesSim'
        local jointStateName='jointStateSim'

        -- Create the subscribers
        MotorSub=simROS.subscribe('/'..MotorTopicName,'std_msgs/Float32MultiArray','setMotorPositions_cb')

        -- Create the publishers
        terminateControllerPub=simROS.advertise('/'..terminateControllerName,'std_msgs/Bool')
        jointPositionsPub=simROS.advertise('/'..jointPositionsName,'std_msgs/Float32MultiArray')
        jointTorquesPub=simROS.advertise('/'..jointTorquesName,'std_msgs/Float32MultiArray')
        jointVelocitiesPub=simROS.advertise('/'..jointVelocitiesName,'std_msgs/Float32MultiArray')
        jointStatePub=simROS.advertise('/'..jointStateName,'std_msgs/Float32MultiArray')


        simStepDoneSub=simROS.subscribe('/simulationStepDone', 'std_msgs/Bool', 'simulationStepDone_cb')
        clockPub=simROS.advertise('/clock','rosgraph_msgs/Clock')
    end

    simROS.setParamBool('use_sim_time', false)
    simROS.publish(terminateControllerPub,{data=false})
    sim.setBoolParameter(sim.boolparam_rosinterface_donotrunmainscript,false)
end

function sysCall_actuation()
    simROS.publish(terminateControllerPub,{data=false})
    simROS.publish(clockPub,{clock=simGetSimulationTime()})
end

function simulationStepDone_cb(msg)
    position_array = { sim.getJointPosition(J0), sim.getJointPosition(J1) }
    torque_array = { sim.getJointForce(J0), sim.getJointForce(J1) }
    
    JOINT_FLOAT_PARAM_ID = 2012
    res, w0 = sim.getObjectFloatParameter(J0, JOINT_FLOAT_PARAM_ID)
    res, w1 = sim.getObjectFloatParameter(J1, JOINT_FLOAT_PARAM_ID)
    velocity_array = { w0, w1 }

    simROS.publish(jointPositionsPub, {data=position_array})
    simROS.publish(jointVelocitiesPub, {data=velocity_array})
    simROS.publish(jointTorquesPub, {data=torque_array})

    data = {}
    lists = {}
    lists[1] = position_array
    lists[2] = velocity_array
    lists[3] = torque_array

    for _, l in pairs(lists) do
        for _, v in pairs(l) do
            table.insert(data, v)
        end
    end

    simROS.publish(jointStatePub, {data=data})
end

function sysCall_cleanup() 
    print('terminating')
    -- Close ROS related stuff
    if not pluginNotFound then
        -- Send termination signal to external ROS nodes
        simROS.publish(terminateControllerPub,{data=true})

        -- Terminate remaining local notes
        simROS.shutdownSubscriber(MotorSub)
        simROS.shutdownPublisher(clockPub)
        simROS.shutdownPublisher(jointPositionsPub)
        simROS.shutdownPublisher(terminateControllerPub)
    end
    print('Lua child script stopped.')
end
