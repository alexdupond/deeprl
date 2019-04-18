import gym
from gym import register
from gym import envs
import gym_dyn_reacher
import gym_ros_reacher

Dyn = False
ROS = False
for envs in envs.registry.all():
    if envs.id == 'DynReacherVelChange-v0':
        Dyn = True
    if envs.id == 'ROSDynReacherVelChange-v0':
        ROS = True
    print(envs.id)

if Dyn:
    print('\nSuccesfull registred!\n DynReacherVelChange-v0 ')
else:
    print('\n :( NOPE!! DynReacherVelChange-v0')

if ROS:
    print('\nSuccesfull registred!\n ROSDynReacherVelChange-v0\n')
else:
    print('\n :( NOPE!! ROSDynReacherVelChange-v0\n')

print("Make env test:\n")
gym.make('DynReacherVelChange-v0')
