#!/bin/bash

if [ "$1" == "real" ]
then
	echo training real robot
	env="ROSForce-v0"
elif [ "$1" == "sim" ]
then
	echo training sim
	env="DynReacherForce-v0"
else
	echo nooo
	exit 1
fi

python3 -m baselines.run --env=$env --alg=ppo2 --env_type=mujoco --network=mlp --nsteps=256 --nminibatches=32 --lam=0.95 --gamma=0.99 --noptepochs=10 --log_interval=1 --ent_coef=0.0 --lr=3e-4 --cliprange=0.2 --value_network=copy
