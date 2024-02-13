import logging
import gymnasium as gym
from gymnasium.envs.registration import register
__version__ = '1.1'

register(
    id='art_wpts-v0',
    entry_point='gym_chrono.envs:art_wpts')  # NAme of the CLASS after the colon
