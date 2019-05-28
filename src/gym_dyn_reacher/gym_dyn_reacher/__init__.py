#import logging
from gym.envs.registration import register

#logger = logging.getLogger(__name__)

register(
    id='DynReacherVelChange-v0',
    entry_point='gym_dyn_reacher.envs:DynReacherVelChangeEnv',
    max_episode_steps=500,
)

register(
    id='DynReacherForce-v0',
    entry_point='gym_dyn_reacher.envs:DynReacherForceEnv',
    max_episode_steps=200,
)

register(
    id='Random-v0',
    entry_point='gym_dyn_reacher.envs:ForceRandomizeEnv',
    max_episode_steps=200,
)
