import pychrono as chrono
from pychrono import irrlicht as chronoirr
import numpy as np
from gym import core, spaces
from collections import OrderedDict


class ChronoBaseEnv(object):
   def __init__(self):
      self.render_setup = False
      return self.get_ob()

   def step(self, ac):
       
       raise NotImplementedError
       

              

   def get_ob(self):
           

          raise NotImplementedError

                 
   def is_done(self):
          raise NotImplementedError
   
   def ScreenCapture(self, interval):
          try: 
              self.myapplication.SetVideoframeSave(True)
              self.myapplication.SetVideoframeSaveInterval(interval)
              
          except:
                 print('No ChIrrApp found. Cannot save video frames.')
                 
                 
   def Render(self):
         raise NotImplementedError

    
    
   def convert_observation_to_space(self, observation):
        if isinstance(observation, dict):
            space = spaces.Dict(OrderedDict([
                (key, self.convert_observation_to_space(value))
                for key, value in observation.items()
            ]))
        elif isinstance(observation, np.ndarray):
            low = np.full(observation.shape, -float('inf'))
            high = np.full(observation.shape, float('inf'))
            space = spaces.Box(low, high, dtype=observation.dtype)
        else:
            raise NotImplementedError(type(observation), observation)
    
        return space

 
   def _set_observation_space(self, observation):
        self.observation_space = self.convert_observation_to_space(observation)
        return self.observation_space
    
   def __del__(self):
        if self.render_setup:
            self.myapplication.GetDevice().closeDevice()
            print('Destructor called, Device deleted.')
        else:
            print('Destructor called, No device to delete.')
        
