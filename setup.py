from setuptools import setup, find_packages

from gym_chrono import __version__

setup(
        name='gym_chrono',
        version=__version__,
        
        url='https://github.com/projectchrono/gym-chrono/tree/demo/art_path_tracking',
        author='UW Simulation Based Engineering Lab',

        packages=find_packages(),
        )
