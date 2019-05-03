import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)

register(
    id='ROSDynReacherVelChange-v0',
    entry_point='gym_ros_reacher_env.envs:ROSDynEnv',
    max_episode_steps=200,
)

register(
    id='ROSForce-v0',
    entry_point='gym_ros_reacher_env.envs:ROSDynForceEnv',
    max_episode_steps=200,
)
