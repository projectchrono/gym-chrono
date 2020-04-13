import numpy as np
import camera_rccar_hallway, camera_cone_track, solo_off_road, camera_obstacle_avoidance
import time

numep = 1

# env = camera_rccar_hallway.camera_rccar_hallway()
env = camera_obstacle_avoidance.camera_obstacle_avoidance()
# env = camera_cone_track.camera_cone_track()
# env = solo_off_road.solo_off_road()

env.reset()

for i in range(numep):
    env.reset()
    # env.play_mode = True
    done = False
    i = 0
    very_start = time.time()
    while not done:
        start = time.time()
        ac = 2* (np.random.rand(3) - 0.5)
        ac[0] = .1
        ac[1] = 1
        ac[2] = 0
        # env.render()
        ob, rew, done, _  = env.step(ac)
        print(i, time.time() - start)
        i += 1
    print('Full Time :: ', time.time() - very_start)
