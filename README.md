**Status:** Under Active Development (New Environments and features will be added)

# gym-chrono

Gym Chrono is a set of continuous state and action spaces DRL environmentbased on the open-source physics engine [Project Chrono](https://projectchrono.org/). 
In order to run these environment you need to install [PyChrono](https://projectchrono.org/pychrono/). 
Being part of Project Chrono, PyChrono is free and open-source. Moreover, it provides an [Anaconda installer](https://anaconda.org/projectchrono/pychrono).

Currently, these tasks are supported:
**chrono_pendulum-v0** 
![](http://projectchrono.org/assets/manual/Tutorial_tensorflow_pendulum.jpg)

Reverse pendulum, the goal is to balance a pole on a cart.  1 action (force along the z axis) and 4 observations (position and speed of cart and pole).

**chrono_ant-v0** 
![](http://projectchrono.org/assets/manual/Tutorial_tensorflow_ant.jpg)
A 4-legged walker, the goal is learning to walk straight as fast as possible. 8 actions (motor torques) and 30 observations (GOG height, COG speed, COG orientation in Euler Angles, COG rotational speed, joints rotation, joints speed, feet contact).

**chrono_hexapod-v0** 
![](http://projectchrono.org/assets/manual/Tutorial_tensorflow_ant.jpg)
A 6-legged walker, the goal is learning to walk straight as fast as possible. Heach legs counts 3 actuated joints.
18 actions (motor torques) and 53 observations (GOG height, COG speed, COG orientation in Euler Angles, COG rotational speed, joints rotation, joints speed, feet contact).

**ChronoRacer3Reach-v0** 
![](http://projectchrono.org/assets/manual/Tutorial_tensorflow_ant.jpg)
A 6-DOF robotic arm, the goal is minimizing the distance between the center of the gripper and the center of the red target box. CAD files and part of the techical were retrieved from  [here](https://www.comau.com/IT/le-nostre-competenze/robotics/robot-team/racer-3-063) .
6 actions (motor torques) and 18 observations (joints rotation, joints speed, end-effector position, target position).

# Installation

```bash
git clone https://github.com/Benatti1991/gym-chrono
cd gym-chrono
pip install -e .
```
