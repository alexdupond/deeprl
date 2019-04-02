rosInitAttempted=false
rosInterfaceSyncModeEnabled=false
pluginFound=false

function attemptRosInit()
    local moduleName=0
    local moduleVersion=0
    local index=0

    while moduleName do
        moduleName,moduleVersion=sim.getModuleName(index)
        if (moduleName=='RosInterface') then
            pluginFound=true
        end
        index=index+1
    end

    if pluginFound then
        startSub=simROS.subscribe('/startSimulation', 'std_msgs/Bool', 'startSimulation_callback')
        pauseSub=simROS.subscribe('/pauseSimulation', 'std_msgs/Bool', 'pauseSimulation_callback')
        stopSub=simROS.subscribe('/stopSimulation', 'std_msgs/Bool', 'stopSimulation_callback')
        enableSyncModeSub=simROS.subscribe('/enableSyncMode', 'std_msgs/Bool', 'enableSyncMode_callback')
        triggerNextStepSub=simROS.subscribe('/triggerNextStep', 'std_msgs/Bool', 'triggerNextStep_callback')

        simStepDonePub=simROS.advertise('/simulationStepDone', 'std_msgs/Bool')
        simStatePub=simROS.advertise('/simulationState','std_msgs/Int32')

        -- aux pub: crazy hack
        auxPub=simROS.advertise('/privateMsgAux', 'std_msgs/Bool')
        auxSub=simROS.subscribe('/privateMsgAux', 'std_msgs/Bool', 'aux_callback')
    else
        sim.displayDialog('Error','The RosInterface was not found.',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
    end

    attemptedRosInit=true
end

function sysCall_init()
    -- might be needed to enable the script
end

function startSimulation_callback(msg)
    sim.startSimulation()
end

function pauseSimulation_callback(msg)
    sim.pauseSimulation()
end

function stopSimulation_callback(msg)
    sim.stopSimulation()
end

function enableSyncMode_callback(msg)
    rosInterfaceSyncModeEnabled=msg.data
    sim.setBoolParameter(sim.boolparam_rosinterface_donotrunmainscript,rosInterfaceSyncModeEnabled)
end

function triggerNextStep_callback(msg)
    sim.setBoolParameter(sim.boolparam_rosinterface_donotrunmainscript,false)
end

function aux_callback(msg)
    simROS.publish(simStepDonePub,{data=true})
end

function publishSimState()
    local state=0 -- simulation not running
    local s=sim.getSimulationState()
    if s==sim.simulation_paused then
        state=2 -- simulation paused
    elseif s==sim.simulation_stopped then
        state=0 -- simulation stopped
    else
        state=1 -- simulation running
    end
    simROS.publish(simStatePub,{data=state})
end

function sysCall_nonSimulation()
    if pluginFound then
        publishSimState()
    elseif not attemptedRosInit then
        attemptRosInit()
    end    
end

function sysCall_actuation()
    if pluginFound then
        publishSimState()
    end
end

function sysCall_sensing()
    if pluginFound then
        simROS.publish(auxPub,{data=true})
        sim.setBoolParameter(sim.boolparam_rosinterface_donotrunmainscript,rosInterfaceSyncModeEnabled)
    end
end

function sysCall_suspended()
    if pluginFound then
        publishSimState()
    end
end

function sysCall_afterSimulation()
    if pluginFound then
        publishSimState()
    end
end

function sysCall_cleanup()
    if pluginFound then
        simROS.shutdownSubscriber(startSub)
        simROS.shutdownSubscriber(pauseSub)
        simROS.shutdownSubscriber(stopSub)
        simROS.shutdownSubscriber(enableSynModeSub)
        simROS.shutdownSubscriber(triggerNextStepSub)
        simROS.shutdownPublisher(simStepDonePub)
        simROS.shutdownPublisher(simStatePub)
    end
end
