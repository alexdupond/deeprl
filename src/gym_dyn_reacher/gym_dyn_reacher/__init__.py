#import logging
from gym.envs.registration import register

#logger = logging.getLogger(__name__)

register(
    id='DynReacherVelChange-v0',
    entry_point='gym_dyn_reacher.envs:DynReacherVelChangeEnv',
    max_episode_steps=500,
)
