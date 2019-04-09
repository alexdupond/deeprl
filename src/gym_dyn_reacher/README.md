# gym_dyn_reacher
A dynamixel reacher mujoco gym


### installation
* [Install mujoco and mujoco-py](https://github.com/openai/mujoco-py)

* Install openai gym: ```pip3 install gym```

* Install gym_dyn_reacher as a python3 package:  
cd into this folder and type ```pip3 install -e .```

If you want to train a model using openai's baseline algorithms:
* Make sure tensorflow is installed ```pip3 install tensorflow``` or ```pip3 install tensorflow-gpu```
* Clone the [baseline repo](https://github.com/openai/baselines)
* gym_dyn_reacher needs to be registered inside run.py for it to find our invironment. In baselines/baselines/run.py, add ```import gym_dyn_reacher``` after ```import gym```.
* Install baselines as a python3 package:
cd into baselines root folder and type ```pip3 install -e .```

You should now be able to run a training:
```
python3 -m baselines.run --alg=ppo2 --env=DynReacherVelChange-v0 --network=mlp --num_timesteps=1e6 --save_path=[path to save the model]
```

and test the model:
```
python3 -m baselines.run --alg=ppo2 --env=DynReacherVelChange-v0 --network=mlp --num_timesteps=0 --load_path=[path to saved model]
```
