import logging
from gym.envs.registration import register

register(
    id='chrono_pendulum-v0',
    entry_point='gym_chrono.envs:ChronoPendulum') # NAme of the CLASS after the colon

register(
    id='chrono_pendulum-v1',
    entry_point='gym_chrono.envs:ChronoPendulumMS') # NAme of the CLASS after the colon

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
)

register(
    id='camera_obstacle_avoidance-v1',
    entry_point='gym_chrono.envs:multisens_obst_avoid'
)

register(
    id='rccar_hallway-v0',
    entry_point='gym_chrono.envs:rccar_hallway'
)

register(
    id='rccar_cone_track-v0',
    entry_point='gym_chrono.envs:rccar_cone_track'
)

register(
    id='barrier_track-v0',
    entry_point='gym_chrono.envs:barrier_track'
)

register(
    id='off_road-v0',
    entry_point='gym_chrono.envs:off_road'
)

register(
    id='gvsets-v0',
    entry_point='gym_chrono.envs:GVSETS_env'
)

register(
    id='off_road_gator-v1',
    entry_point='gym_chrono.envs:off_road_gator_v1'
)

register(
    id='off_road_gator-v0',
    entry_point='gym_chrono.envs:off_road_gator'
)
register(
    id='off_road_gator-v2',
    entry_point='gym_chrono.envs:off_road_gator_v2'
)

register(
    id='robot_learning-v0',
    entry_point='gym_chrono.envs:robot_learning'
    #timestep_limit=1000,
    #reward_threshold=10.0,
    #nondeterministic = True,
)
