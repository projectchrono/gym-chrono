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
)

register(
    id='ChronoRacer3Reach-v0',
    entry_point='gym_chrono.envs:ChronoComauR3'
)

register(
    id='camera_obstacle_avoidance-v0',
    entry_point='gym_chrono.envs:camera_obstacle_avoidance'
)

register(
    id='rccar_hallway-v0',
    entry_point='gym_chrono.envs:rccar_hallway'
)

register(
    id='barrier_track-v0',
    entry_point='gym_chrono.envs:barrier_track'
)

register(
    id='convoy-v0',
    entry_point='gym_chrono.envs:convoy'
)

register(
    id='off_road_gator-v0',
    entry_point='gym_chrono.envs:off_road_gator'
)
register(
    id='off_road_gator-v3',
    entry_point='gym_chrono.envs:off_road_gator_v3'
)

