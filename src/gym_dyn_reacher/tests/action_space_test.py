from gym_dyn_reacher.envs.DynReacherForceEnv import DynReacherForceEnv
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--freq", type=int, required=True)
parser.add_argument("--delay", type=float, required=True)
parser.add_argument("--dist_rew", type=float, required=True)
parser.add_argument("--elec_rew", type=float, required=True)
parser.add_argument("--stuck_rew", type=float, required=True)
args = parser.parse_args()


env = DynReacherForceEnv(
    frequency=args.freq,
    delay=args.delay,
    distance_reduction_reward_weight=args.dist_rew,
    electricity_reward_weight=args.elec_rew,
    stuck_joint_reward_weight=args.stuck_rew,
)

env.reset()

r = 0
i = 0
while True:
    action = env.action_space.sample()
    env.render()
    obs, reward, done, info = env.step(action)  # take a random action
    r = 0.9 * r + 0.1 * reward
    if i % 10 == 0:
        print('{:.2f}'.format(r))
    i += 1
    if done or i > 300:
        env.reset()
        i = 0
        r = 0
