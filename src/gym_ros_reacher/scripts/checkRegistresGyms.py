#!/usr/bin/env python3
import os
import gym
from gym import register
from gym import envs
try:
    import gym_dyn_reacher
except ImportError:
    print("Couldn't import gym_dyn_reacher")
try:
    import gym_ros_reacher_env
except ImportError:
    print("Couldn't import gym_ros_reacher_env")

from gym.envs.registration import register

Dyn = False
ROS = False
for env in envs.registry.all():
    if env.id == 'DynReacherVelChange-v0':
        Dyn = True
    if env.id == 'ROSDynReacherVelChange-v0':
        ROS = True
    #print(envs.id)

if Dyn:
    print('\nSuccesfull registred!\n DynReacherVelChange-v0 ')
else:
    print('\n :( NOPE!! DynReacherVelChange-v0')

if ROS:
    print('\nSuccesfull registred!\n ROSDynReacherVelChange-v0\n')
else:
    print('\n :( NOPE!! ROSDynReacherVelChange-v0\n')

print("Make env test:\n")
env = gym.make('ROSDynReacherVelChange-v0')
observation = env.reset()
print('Did a observation: \n',observation)

print("OS home: ")
print(os.environ['HOME'])
import sys
print(sys.prefix)
