#!/bin/bash

python3 -m baselines.run --env=ROSForce-v0 --alg=ppo2 --network=mlp --nsteps=128 --nminibatches=32 --lam=0.95 --gamma=0.99 --noptepochs=10 --log_interval=1 --ent_coef=0.0 --lr=3e-4 --cliprange=0.2 --value_network=copy
