import pychrono as chrono
try:
    import pychrono.irrlicht as chronoirr
except:
    print('Could not import ChronoIrrlicht')

import numpy as np

# gym imports
import gymnasium as gym
import os


class ChronoBaseEnv(gym.Env):
    """
    Base class for Chrono environments.
    """

    def __init__(self, render_mode='human'):
        # Data subdirectory in this folder
        self.chronopath = os.path.join(os.path.dirname(__file__), 'data/')
        chrono.SetChronoDataPath(self.chronopath)
        self.render_mode = render_mode
        self.render_setup = False

        # Used to SetNumThreads
        self._cpu = [1, 1, 1]

        return

    @property
    def cpu(self):
        return self._cpu

    def step(self, action):
        """
        Perform a simulation step of the environment.
        :param action: The action to apply to the environment.
        :return: The observation, the reward, a boolean indicating if the episode is terminated, a boolean indicating if the episode is truncated, and a dictionary of info.
        """
        raise NotImplementedError

    def reset(self, seed=None, options=None):
        """
        Reset the environment to its initial state.
        :param seed (Optional): The seed to use for the simulation.
        :param options (Optional): The options to pass to the simulation.
        :return: The observation, and the dictionary of info.
        """
        raise NotImplementedError

    def render(self):
        """
        Render the environment.
        """
        raise NotImplementedError

    def get_observation(self):
        """
        Get the current observation.
        :return: The current observation.
        """
        raise NotImplementedError

    def is_terminated(self):
        """
        Check if the episode is terminated.
        :return: Return true indicating if the episode is terminated.
        """
        raise NotImplementedError

    def is_truncated(self):
        """
        Check if the episode is truncated.
        :return: Return true indicating if the episode is truncated.
        """
        raise NotImplementedError

    def convert_observation_to_gymspace(self, obs):
        """
        Convert the observation to the gym space.
        :param obs: The observation to convert.
        :return: The converted observation.
        """
        raise NotImplementedError

    def _set_observation_space(self, observation):
        """
        Set the observation space.
        :param observation: The observation to set the observation space.
        """
        self.observation_space = self.convert_observation_to_gymspace(
            observation)
        return self.observation_space

    def ScreenCapture(self, interval):
        """
        Enable saving screen and set the screen capture interval.
        :param interval: The interval to set the screen capture.
        """
        try:
            self.myapplication.SetVideoframeSave(True)
            self.myapplication.SetVideoframeSaveInterval(interval)

        except:
            print('No ChIrrApp found. Cannot save video frames.')

    def __del__(self):
        if self.render_setup:
            self.myapplication.GetDevice().closeDevice()
            print('Destructor called, Device deleted.')
        else:
            print('Destructor called, No device to delete.')

    def __setstate__(self, state):
        self.__init__()
        return {self}

    def __getstate__(self):

        return {}
