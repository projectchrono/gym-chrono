import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)

register(
    id='ChronoPendulum',
    entry_point='gym_chrono.envs:chrono_pendulum',
    #timestep_limit=1000,
    #reward_threshold=1.0,
    #nondeterministic = True,
)

register(
    id='ChronoAnt',
    entry_point='gym_chrono.envs:chrono_ant',
    #timestep_limit=1000,
    #reward_threshold=10.0,
    #nondeterministic = True,
)

register(
    id='ChronoHexapod',
    entry_point='gym_chrono.envs:chrono_Hexapod',
    #timestep_limit=1000,
    #reward_threshold=1.0,
    #nondeterministic = True,
)

register(
    id='ChronoRacer3Reach',
    entry_point='gym_chrono.envs:chrono_ComauR3',
    #timestep_limit=1000,
    #reward_threshold=10.0,
    #nondeterministic = True,
)
