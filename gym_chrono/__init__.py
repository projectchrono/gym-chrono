import logging
from gym.envs.registration import register

register(
    id='chrono_pendulum-v0',
    entry_point='gym_chrono.envs:ChronoPendulum') # NAme of the CLASS after the colon

register(
    id='chrono_ant-v0',
    entry_point='gym_chrono.envs:ChronoAnt'
)

register(
    id='chrono_hexapod-v0',
    entry_point='gym_chrono.envs:ChronoHexapod'
    #timestep_limit=1000,
    #reward_threshold=1.0,
    #nondeterministic = True,
)

register(
    id='ChronoRacer3Reach-v0',
    entry_point='gym_chrono.envs:ChronoComauR3'
    #timestep_limit=1000,
    #reward_threshold=10.0,
    #nondeterministic = True,
)

register(
    id='camera_obstacle_avoidance-v0',
    entry_point='gym_chrono.envs:camera_obstacle_avoidance'
    #timestep_limit=1000,
    #reward_threshold=10.0,
    #nondeterministic = True,
)

register(
    id='camera_obstacle_avoidance-v1',
    entry_point='gym_chrono.envs:multisens_obst_avoid'
    #timestep_limit=1000,
    #reward_threshold=10.0,
    #nondeterministic = True,
)

register(
    id='camera_rccar_hallway-v0',
    entry_point='gym_chrono.envs:camera_rccar_hallway'
    #timestep_limit=1000,
    #reward_threshold=10.0,
    #nondeterministic = True,
)

register(
    id='camera_cone_track-v0',
    entry_point='gym_chrono.envs:camera_cone_track'
    #timestep_limit=1000,
    #reward_threshold=10.0,
    #nondeterministic = True,
)

register(
    id='camera_barrier_track-v0',
    entry_point='gym_chrono.envs:camera_barrier_track'
    #timestep_limit=1000,
    #reward_threshold=10.0,
    #nondeterministic = True,
)

register(
    id='solo_off_road-v0',
    entry_point='gym_chrono.envs:solo_off_road'
    #timestep_limit=1000,
    #reward_threshold=10.0,
    #nondeterministic = True,
)
