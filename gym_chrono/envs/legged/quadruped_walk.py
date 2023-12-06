# =======================================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2021 projectchrono.org
# All right reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =======================================================================================
# Authors: Jason Zhou
# =======================================================================================

#
# =======================================================================================
# =======================================================================================

# Chrono imports
import pychrono as chrono
import pychrono.robot as robot
import cmath
try:
    from pychrono import irrlicht as chronoirr
except:
    print('Could not import ChronoIrrlicht')
try:
    import pychrono.sensor as sens
except:
    print('Could not import Chrono Sensor')

try:
    from pychrono import irrlicht as chronoirr
except:
    print('Could not import ChronoIrrlicht')


# Gym chrono imports
# Custom imports
from gym_chrono.envs.ChronoBase import ChronoBaseEnv
from gym_chrono.envs.utils.utils import CalcInitialPose, chVector_to_npArray, npArray_to_chVector, SetChronoDataDirectories

# Standard Python imports
import os
import math
import numpy as np

# Gymnasium imports
import gymnasium as gym


class quadruped_walk(ChronoBaseEnv):
    """
    Wrapper for the cobra chrono model into a gym environment.
    Mainly built for use with action space = 
    """

    def __init__(self, render_mode='human'):
        ChronoBaseEnv.__init__(self, render_mode)

        SetChronoDataDirectories()
        # Define action space 
        self.action_space = gym.spaces.Box(
            low=-3, high=3, shape=(12,), dtype=np.float64)

        # Define observation space
        self.observation_space = gym.spaces.Box(
            low=-30, high=30, shape=(18,), dtype=np.float64)

        # -----------------------------
        # Chrono simulation parameters
        # -----------------------------
        self.system = None  # Chrono system set in reset method
        self.ground = None  # Ground body set in reset method
        self.unitree = None  # Robot set in reset method
   
        # Frequncy in which we apply control
        self._control_frequency = 20
        # Dynamics timestep
        self._step_size = 5e-4
        self._sim_time = 0.0
        # Number of steps dynamics has to take before we apply control
        self._steps_per_control = round(
            1 / (self._step_size * self._control_frequency))
        
        self._prev_pos_x = 0

        # ---------------------------------
        # Gym Environment variables
        # ---------------------------------
        # Maximum simulation time (seconds)
        self._max_time = 50
        # Holds reward of the episode
        self.reward = 0
        self._debug_reward = 0
        # Observation of the environment
        self.observation = None
        # Flag to determine if the environment has terminated -> In the event of timeOut or reach goal
        self._terminated = False
        # Flag to determine if the environment has truncated -> In the event of a crash
        self._truncated = False
        # Flag to check if the render setup has been done -> Some problem if rendering is setup in reset
        self._render_setup = False

    def reset(self, seed=None, options=None):
        self._prev_pos_x = 0
        
        # -----------------------------
        # Set up system with collision
        # -----------------------------
        self.system = chrono.ChSystemSMC()
        self.system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
        self.system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
        chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
        chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)

        # -----------------------------
        # Set up Terrain
        # -----------------------------
        # Create ground body
        ground_mat = chrono.ChMaterialSurfaceSMC()
        ground_mat.SetFriction(0.9)
        ground_mat.SetYoungModulus(1e7)
        ground = chrono.ChBodyEasyBox(20, 20, 0.1, 1000, True, True, ground_mat)
        ground.SetPos(chrono.ChVectorD(0, 0, 0))
        ground.SetBodyFixed(True)
        ground.SetCollide(True)
        ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
        self.system.Add(ground)

        # -----------------------------
        # Create the Robot
        # -----------------------------
        self.unitree = robot.Unitree_Go1(self.system)
        self.unitree.Initialize(chrono.ChFrameD(chrono.ChVectorD(0, 0.0, 0.5), chrono.ChQuaternionD(1, 0, 0, 0)))

        # -----------------------------
        # Get the intial observation
        # -----------------------------
        self.observation = self.get_observation()
        # _vector_to_goal is a chrono vector
        self._debug_reward = 0

        self._terminated = False
        self._truncated = False
        self._success = False

        return self.observation, {}

    def step(self, action):
        self._sim_time = self.system.GetChTime()
    
        
        # Apply the actions
        self.unitree.SetHipMotorPos(0, action[0], self._sim_time)
        self.unitree.SetHipMotorPos(1, action[1], self._sim_time)
        self.unitree.SetHipMotorPos(2, action[2], self._sim_time)
        self.unitree.SetHipMotorPos(3, action[3], self._sim_time)
        
        self.unitree.SetThighMotorPos(0, action[4], self._sim_time)
        self.unitree.SetThighMotorPos(1, action[5], self._sim_time)
        self.unitree.SetThighMotorPos(2, action[6], self._sim_time)
        self.unitree.SetThighMotorPos(3, action[7], self._sim_time)
        
        self.unitree.SetCalfMotorPos(0, action[8], self._sim_time)
        self.unitree.SetCalfMotorPos(1, action[9], self._sim_time)
        self.unitree.SetCalfMotorPos(2, action[10], self._sim_time)
        self.unitree.SetCalfMotorPos(3, action[11], self._sim_time)

        for i in range(self._steps_per_control):
            self._sim_time = self.system.GetChTime()
            self.system.DoStepDynamics(self._step_size)

        # Get the observation
        self.observation = self.get_observation()
        # Get reward
        self.reward = self.get_reward()
        self._debug_reward += self.reward
        # Check if we are done
        self._is_terminated()
        self._is_truncated()

        return self.observation, self.reward, self._terminated, self._truncated, {}

    def render(self, mode='human'):
        """
        Render the environment
        """

        # ------------------------------------------------------
        # Add visualization - only if we want to see "human" POV
        # ------------------------------------------------------
        if mode == 'human':
            if self._render_setup == False:
                self.vis = chronoirr.ChVisualSystemIrrlicht()
                self.vis.AttachSystem(self.system)
                self.vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
                self.vis.SetWindowSize(1280, 720)
                self.vis.SetWindowTitle('Cobro RL playground')
                self.vis.Initialize()
                self.vis.AddSkyBox()
                self.vis.AddCamera(chrono.ChVectorD(
                    1.5, 2, 2), chrono.ChVectorD(0, 0, 0.5))
                self.vis.AddTypicalLights()
                self.vis.AddLightWithShadow(chrono.ChVectorD(
                    1.5, -50, 100), chrono.ChVectorD(0, 0, 0.5), 3, 4, 10, 40, 512)
                self._render_setup = True

            self.vis.BeginScene()
            self.vis.Render()
            self.vis.EndScene()
        else:
            raise NotImplementedError

    def get_reward(self):
        reward = 0.0
        
        reward = (self.unitree.GetTrunkBody().GetPos().x - self._prev_pos_x)
        self._prev_pos_x = self.unitree.GetTrunkBody().GetPos().x
        
        return reward

    def _is_terminated(self):
        
        if self._sim_time > self._max_time:
            self._terminated = True
        else:
            self._terminated = False

    def _is_truncated(self):
        
        
        if self.unitree.GetTrunkBody().GetPos().z < 0.15 or self.unitree.GetTrunkBody().GetPos().z > 1.0 or self.unitree.GetTrunkBody().GetPos().y < -1.5 or self.unitree.GetTrunkBody().GetPos().y > 1.5:
            self._truncated = True
            self.reward -= 200
            self._debug_reward -= 200
        else: 
            self._truncated = False


    def get_observation(self):
        observation = np.zeros(18)
        
        # Get current motor positions
        observation[0] = self.unitree.GetHipMotorPos(0, self._sim_time)
        observation[1] = self.unitree.GetHipMotorPos(1, self._sim_time)
        observation[2] = self.unitree.GetHipMotorPos(2, self._sim_time)
        observation[3] = self.unitree.GetHipMotorPos(3, self._sim_time)
        
        observation[4] = self.unitree.GetThighMotorPos(0, self._sim_time)
        observation[5] = self.unitree.GetThighMotorPos(1, self._sim_time)
        observation[6] = self.unitree.GetThighMotorPos(2, self._sim_time)
        observation[7] = self.unitree.GetThighMotorPos(3, self._sim_time)
        
        observation[8] = self.unitree.GetCalfMotorPos(0, self._sim_time)
        observation[9] = self.unitree.GetCalfMotorPos(1, self._sim_time)
        observation[10] = self.unitree.GetCalfMotorPos(2, self._sim_time)
        observation[11] = self.unitree.GetCalfMotorPos(3, self._sim_time)
        
        # Get the position of the robot
        robot_pos = self.unitree.GetTrunkBody().GetPos()
        observation[12] = robot_pos.x
        observation[13] = robot_pos.y
        observation[14] = robot_pos.z
        
        # Get the orientation of the robot
        robot_rot = self.unitree.GetTrunkBody().GetRot()
        robot_rot = robot_rot.Q_to_Euler123()
        observation[15] = robot_rot.x
        observation[16] = robot_rot.y
        observation[17] = robot_rot.z

        # For not just the priveledged position of the robot
        return observation
