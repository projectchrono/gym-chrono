**Status:** Under Active Development (New Environments and features will be added)
> :warning: **These environment are compatible with the develop version of ChronoAPI**: Checkout develop branch if you build from [sources](https://github.com/projectchrono/chrono) or use develop label if you install with [Anaconda](https://anaconda.org/projectchrono/pychrono)
# gym-chrono

Gym Chrono is a set of continuous state and action spaces DRL environmentbased on the open-source physics engine [Project Chrono](https://projectchrono.org/). 
In order to run these environment you need to install [PyChrono](https://projectchrono.org/pychrono/). 
Being part of Project Chrono, PyChrono is **free and open-source**. Moreover, it provides an [Anaconda installer](https://anaconda.org/projectchrono/pychrono).
Currently, these tasks are supported:

**chrono_pendulum-v0** 


![](http://projectchrono.org/assets/manual/Tutorial_tensorflow_pendulum.jpg)

Reverse pendulum, the goal is to balance a pole on a cart.  1 action (force along the z axis) and 4 observations (position and speed of cart and pole).

**chrono_ant-v0** 


![](http://projectchrono.org/assets/manual/Tutorial_tensorflow_ant.jpg)

A 4-legged walker, the goal is learning to walk straight as fast as possible. 8 actions (motor torques) and 30 observations (GOG height, COG speed, COG orientation in Euler Angles, COG rotational speed, joints rotation, joints speed, feet contact).

**chrono_hexapod-v0** 


![](https://github.com/projectchrono/chrono-web-assets/blob/master/Images/Hexapod.jpg)

A 6-legged walker, the goal is learning to walk straight as fast as possible. Heach legs counts 3 actuated joints.
18 actions (motor torques) and 53 observations (GOG height, COG speed, COG orientation in Euler Angles, COG rotational speed, joints rotation, joints speed, feet contact).

This is a model of a real hexapod robot, the PhantomX Hexapod Mark II. Motors dara are available [here](https://trossenrobotics.com/dynamixel-ax-12-robot-actuator.aspx). Credit to [Hendricks](https://grabcad.com/hendricks-1) for the CAD drawings.

**ChronoRacer3Reach-v0** 


![](https://github.com/projectchrono/chrono-web-assets/blob/master/Images/Comau.jpg)

A 6-DOF robotic arm (Comau Racer-3), the goal is minimizing the distance between the center of the gripper and the center of the red target box. CAD files and part of the techical data are freely available from the Comau website [here](https://www.comau.com/IT/le-nostre-competenze/robotics/robot-team/racer-3-063) .
6 actions (motor torques) and 18 observations (joints rotation, joints speed, end-effector position, target position).

# Installation
If you want to install Gym and Chrono in a virtual environment (Conda or VirtualEnv), don't forget to activate the environment first.
To install Chrono follow [these](http://api.projectchrono.org/development/pychrono_installation.html) instructions. 
Install Gym. Obviously, OpenAI Gym is a prerequisite for gym-chrono. 
```bash
pip install gym
```
Then, install the PyChrono extension for Gym:
```bash
git clone https://github.com/projectchrono/gym-chrono
cd gym-chrono
pip install -e .
```

# Example
Training the ant environment using the PPOalgorithm in [OpenAI Baselines](https://github.com/openai/baselines)
```bash
python -m baselines.run --alg=ppo2 --env=gym_chrono.envs:chrono_ant-v0 --network=mlp --num_timesteps=2e7 --ent_coef=0.1 --num_hidden=32 --num_layers=3 --value_network=copy
```


