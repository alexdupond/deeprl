#import logging
from gym.envs.registration import register

#logger = logging.getLogger(__name__)

register(
    id='ROSDynReacherVelChange-v0',
    entry_point='gym_ros_reacher.envs:ROSDynReacherVelChangeEnv',
    max_episode_steps=500,
)
