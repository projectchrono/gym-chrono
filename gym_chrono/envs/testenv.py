import numpy as np
import camera_obstacle_avoidance

numep = 2

env = camera_obstacle_avoidance.camera_obstacle_avoidance()

env.reset()

for i in range(numep):
    env.reset()
    done = False
    while not done:
        ac = 2* (np.random.rand(2) - 0.5)
        env.render()
        ob, rew, done, _  = env.step(ac)
