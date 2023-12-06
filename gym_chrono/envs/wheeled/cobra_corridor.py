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
# Authors: Huzaifa Unjhawala, Jason Zhou
# =======================================================================================
#
# This file contains a gym environment for the cobra rover in a terrain of 20 x 20. The
# environment is used to train the rover to reach a goal point in the terrain. The goal
# point is randomly generated in the terrain. The rover is initialized at the center of
# the terrain. Obstacles can be optionally set (default is 0).
#
# =======================================================================================
#
# Action Space: The action space is normalized throttle and steering between -1 and 1.
# multiply against the max wheel angular velocity and wheel steer angle to provide the
# wheel angular velocity and wheel steer angle for all 4 wheels of the cobra rover model.
# Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float64)
#
# =======================================================================================
#
# Observation Space: The observation space is a 1D array consisting of the following:
# 1. Delta x of the goal in local frame of the vehicle
# 2. Delta y of the goal in local frame of the vehicle
# 3. Vehicle heading
# 4. Heading needed to reach the goal
# 5. Velocity of vehicle
# In case obstacles exist, the observation space also include
# 6. x position of obstacle in local frame of the vehicle
# 7. y position of obstacle in local frame of the vehicle
# 8. radius of obstacle
#
# =======================================================================================


# Chrono imports
import pychrono as chrono
import pychrono.robot as robot_chrono
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


class cobra_corridor(ChronoBaseEnv):
    """
    Wrapper for the cobra chrono model into a gym environment.
    Mainly built for use with action space = 
    """

    def __init__(self, render_mode='human'):
        ChronoBaseEnv.__init__(self, render_mode)

        SetChronoDataDirectories()

        # ----------------------------
        # Action and observation space
        # -----------------------------

        # Max steering in radians
        self.max_steer = np.pi / 6.
        # Max motor speed in radians per sec
        self.max_speed = 2*np.pi

        # self.num_obs = np.random.randint(5, 10)
        self.num_obs = 3
        # Define action space -> These will scale the max steer and max speed linearly
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float64)

        # Define observation space
        # First few elements describe the relative position of the rover to the goal
        # Delta x in local frame
        # Delta y in local frame
        # Vehicle heading
        # Heading needed to reach the goal
        # Velocity of vehicle
        # Then we have the x,y poisition of the obstacles with the obstacle radius
        self.observation_space = gym.spaces.Box(
            low=-20, high=20, shape=(5 + self.num_obs*3,), dtype=np.float64)

        # -----------------------------
        # Chrono simulation parameters
        # -----------------------------
        self.system = None  # Chrono system set in reset method
        self.ground = None  # Ground body set in reset method
        self.rover = None  # Rover set in reset method

        self.x_obs = None
        self.y_obs = None

        self._initpos = chrono.ChVectorD(
            0.0, 0.0, 0.0)  # Rover initial position
        # Frequncy in which we apply control
        self._control_frequency = 10
        # Dynamics timestep
        self._step_size = 1e-3
        # Number of steps dynamics has to take before we apply control
        self._steps_per_control = round(
            1 / (self._step_size * self._control_frequency))
        self._collision = False
        self._terrain_length = 20
        self._terrain_width = 20
        self._terrain_height = 2
        self.vehicle_pos = None

        # ---------------------------------
        # Gym Environment variables
        # ---------------------------------
        # Maximum simulation time (seconds)
        self._max_time = 50
        # Holds reward of the episode
        self.reward = 0
        self._debug_reward = 0
        # Position of goal as numpy array
        self.goal = None
        # Distance to goal at previos time step -> To gauge "progress"
        self._vector_to_goal = None
        self._old_distance = None
        # Observation of the environment
        self.observation = None
        # Flag to determine if the environment has terminated -> In the event of timeOut or reach goal
        self._terminated = False
        # Flag to determine if the environment has truncated -> In the event of a crash
        self._truncated = False
        # Flag to check if the render setup has been done -> Some problem if rendering is setup in reset
        self._render_setup = False
        # Flag to count success while testing
        self._success = False

    def reset(self, seed=None, options=None):
        """
        Reset the environment to its initial state -> Set up for standard gym API
        :param seed: Seed for the random number generator
        :param options: Options for the simulation (dictionary)
        """

        # -----------------------------
        # Set up system with collision
        # -----------------------------
        self.system = chrono.ChSystemNSC()
        self.system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
        chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
        chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

        # -----------------------------
        # Set up Terrain
        # -----------------------------
        ground_mat = chrono.ChMaterialSurfaceNSC()
        self.ground = chrono.ChBodyEasyBox(
            self._terrain_length, self._terrain_width, self._terrain_height, 1000, True, True, ground_mat)
        self.ground.SetPos(chrono.ChVectorD(0, 0, -self._terrain_height / 2))
        self.ground.SetBodyFixed(True)
        self.ground.GetVisualShape(0).SetTexture(
            chrono.GetChronoDataFile('textures/concrete.jpg'), 200, 200)
        self.system.Add(self.ground)

        # -----------------------------
        # Create the COBRA
        # -----------------------------
        self.rover = robot_chrono.Cobra(
            self.system, robot_chrono.CobraWheelType_SimpleWheel)
        self.driver = robot_chrono.CobraSpeedDriver(
            1/self._control_frequency, 0.0)
        self.rover.SetDriver(self.driver)

        # Initialize position of robot randomly
        self.initialize_robot_pos(seed)

        # -----------------------------
        # Add obstacles
        # -----------------------------

        self.add_obstacles(seed)

        # -----------------------------
        # Add sensors
        # -----------------------------
        self.add_sensors()

        # ---------------------------------------
        # Set the goal point and set premilinaries
        # ---------------------------------------
        self.set_goalPoint(seed=1)

        # -----------------------------
        # Get the intial observation
        # -----------------------------
        self.observation = self.get_observation()
        # self._old_distance = np.linalg.norm(self.observation[:3] - self.goal)
        # _vector_to_goal is a chrono vector
        self._old_distance = self._vector_to_goal.Length()
        self._debug_reward = 0

        self._terminated = False
        self._truncated = False
        self._success = False

        return self.observation, {}

    def step(self, action):
        """
        Take a step in the environment - Frequency by default is 10 Hz.
        """

        # Linearly interpolate steer angle between pi/6 and pi/8
        steer_angle = action[0] * self.max_steer
        wheel_speed = action[1] * self.max_speed
        self.driver.SetSteering(steer_angle)  # Maybe we should ramp this steer
        # Wheel speed is ramped up to wheel_speed with a ramp time of 1/control_frequency
        self.driver.SetMotorSpeed(wheel_speed)

        for i in range(self._steps_per_control):
            self.rover.Update()
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
                    0, 11, 10), chrono.ChVectorD(0, 0, 1))
                self.vis.AddTypicalLights()
                self.vis.AddLightWithShadow(chrono.ChVectorD(
                    1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0.5), 3, 4, 10, 40, 512)
                self._render_setup = True

            self.vis.BeginScene()
            self.vis.Render()
            self.vis.EndScene()
        else:
            raise NotImplementedError

    def get_reward(self):
        """
        Get the reward for the current step
        """
        scale_pos = 200
        scale_neg = 200
        # Distance to goal
        # distance = np.linalg.norm(self.observation[:3] - self.goal)
        distance = self._vector_to_goal.Length()  # chrono vector
        if (self._old_distance > distance):
            reward = scale_pos * (self._old_distance - distance)
        else:
            reward = scale_neg * (self._old_distance - distance)

        # If we have not moved even by 1 cm in 0.1 seconds, give penalty
        if (np.abs(self._old_distance - distance) < 0.01):
            reward -= 10

        # Update the old distance
        self._old_distance = distance

        return reward

    def _is_terminated(self):
        """
        Check if the environment is terminated
        """

        # If we are within a certain distance of the goal -> Terminate and give big reward
        # if np.linalg.norm(self.observation[:3] - self.goal) < 0.4:
        if np.linalg.norm(self._vector_to_goal.Length()) < 0.75:
            print('--------------------------------------------------------------')
            print('Goal Reached')
            print('Initial position: ', self._initpos)
            print('Goal position: ', self.goal)
            print('--------------------------------------------------------------')
            self.reward += 2500
            self._debug_reward += self.reward
            self._terminated = True
            self._success = True

        # If we have exceeded the max time -> Terminate
        if self.system.GetChTime() > self._max_time:
            print('--------------------------------------------------------------')
            print('Time out')
            print('Initial position: ', self._initpos)
            # dist = np.linalg.norm(self.observation[:3] - self.goal)
            dist = self._vector_to_goal.Length()
            print('Final position of rover: ',
                  self.rover.GetChassis().GetPos())
            print('Goal position: ', self.goal)
            print('Distance to goal: ', dist)
            # Penalize based on how far we are from the goal
            self.reward -= 100 * dist

            self._debug_reward += self.reward
            print('Reward: ', self.reward)
            print('Accumulated Reward: ', self._debug_reward)
            print('--------------------------------------------------------------')
            self._terminated = True

    def _is_truncated(self):
        """
        Check if the environment is truncated
        """
        # If we have collided -> Truncate and give big negative reward
        self.check_collision()
        if self._collision:
            print('--------------------------------------------------------------')
            print('Crashed')
            print('Vehicle Postion: ', self.vehicle_pos)
            print('Goal Position: ', self.goal)
            print('--------------------------------------------------------------')
            self.reward -= 600
            self._debug_reward += self.reward
            self._truncated = True
        # Vehicle should not fall off the terrain
        elif ((abs(self.vehicle_pos.x) > (self._terrain_length / 2.0 - 0.5)) or (abs(
                self.vehicle_pos.y) > (self._terrain_width / 2. - 0.5)) or (self.vehicle_pos.z < 0)):
            print('--------------------------------------------------------------')
            print('Outside of terrain')
            print('Vehicle Position: ', self.vehicle_pos)
            print('Goal Position: ', self.goal)
            print('--------------------------------------------------------------')
            self.reward -= 600
            self._debug_reward += self.reward
            self._truncated = True

    def initialize_robot_pos(self, seed=1):
        """
        Initialize the pose of the robot
        """
        self._initpos = chrono.ChVectorD(0, -0.2, 0.08144073)

        # For now no randomness
        self.rover.Initialize(chrono.ChFrameD(
            self._initpos, chrono.ChQuaternionD(1, 0, 0, 0)))

        self.vehicle_pos = self._initpos

    def set_goalPoint(self, seed=1):
        """
        Set the goal point for the rover
        """
        # np.random.seed(seed)

        a = -8.
        b = 8.

        redo = True
        while (redo):
            if (self.num_obs == 0):
                goal_pos = a + (b - a) * np.random.rand(2)
                break
            goal_pos = a + (b - a) * np.random.rand(2)
            for i in range(self.num_obs):
                if np.linalg.norm(np.array([self.x_obs[i], self.y_obs[i]]) - goal_pos) < 1:
                    redo = True
                    break
                else:
                    redo = False
            # Check if the point is too close to initial position of the rover
            if abs(goal_pos[0] - self._initpos.x) < 2:
                redo = True
            if abs(goal_pos[1] - self._initpos.y) < 2:
                redo = True

        # Some random goal point for now
        self.goal = np.array([goal_pos[0], goal_pos[1], 0.08144073])
        # print('Goal: ', self.goal)

        # -----------------------------
        # Set up goal visualization
        # -----------------------------
        goal_contact_material = chrono.ChMaterialSurfaceNSC()
        goal_mat = chrono.ChVisualMaterial()
        goal_mat.SetAmbientColor(chrono.ChColor(1., 0., 0.))
        goal_mat.SetDiffuseColor(chrono.ChColor(1., 0., 0.))

        goal_body = chrono.ChBodyEasySphere(
            0.2, 1000, True, False, goal_contact_material)

        goal_body.SetPos(chrono.ChVectorD(
            goal_pos[0], goal_pos[1], 0.2))
        goal_body.SetBodyFixed(True)
        goal_body.GetVisualShape(0).SetMaterial(0, goal_mat)

        self.system.Add(goal_body)

    def get_observation(self):
        """
        Get the observation from the environment
            1. Delta x of the goal in local frame of the vehicle
            2. Delta y of the goal in local frame of the vehicle
            3. Vehicle heading
            4. Heading needed to reach the goal
            5. Velocity of vehicle
            In case obstacles exist, the observation space also include
            6. x position of obstacle in local frame of the vehicle
            7. y position of obstacle in local frame of the vehicle
            8. radius of obstacle
        """
        observation = np.zeros(5 + self.num_obs*3)

        self.vehicle_pos = self.rover.GetChassis().GetPos()
        self._vector_to_goal = npArray_to_chVector(
            self.goal) - self.vehicle_pos
        vector_to_goal_local = self.rover.GetChassis(
        ).GetRot().RotateBack(self._vector_to_goal)

        # Observation features
        vehicle_heading = self.rover.GetChassis().GetRot().Q_to_Euler123().z
        vehicle_velocity = self.rover.GetChassisVel()
        local_delX = vector_to_goal_local.x * \
            np.cos(vehicle_heading) + vector_to_goal_local.y * \
            np.sin(vehicle_heading)
        local_delY = -vector_to_goal_local.x * \
            np.sin(vehicle_heading) + vector_to_goal_local.y * \
            np.cos(vehicle_heading)
        target_heading_to_goal = np.arctan2(
            vector_to_goal_local.y, vector_to_goal_local.x)

        observation[0] = local_delX
        observation[1] = local_delY
        observation[2] = vehicle_heading
        observation[3] = target_heading_to_goal
        observation[4] = vehicle_velocity.Length()

        count = 0
        spaces_filled = 5
        for i in range(spaces_filled, self.num_obs*3+spaces_filled, 3):
            obstacle_pos = chrono.ChVectorD(
                self.x_obs[count], self.y_obs[count], 0.5)
            rel_obstacle_pos_local = self.rover.GetChassis().GetRot(
            ).RotateBack(obstacle_pos - self.vehicle_pos)

            # observation[i] = self.x_obs[count]
            # observation[i + 1] = self.y_obs[count]
            # observation[i + 2] = 1
            observation[i] = rel_obstacle_pos_local.x
            observation[i + 1] = rel_obstacle_pos_local.y
            observation[i + 2] = 1
            count += 1

        # For not just the priveledged position of the rover
        return observation

    # -------------Add Random Objects to the environment -------------------------------------

    def add_obstacles(self, seed=1):
        """
        Add random obstacles to the environment
        """
        # np.random.seed(seed)

        self.x_obs = np.zeros(self.num_obs)
        self.y_obs = np.zeros(self.num_obs)
        # Generate a random float between -8 and 8
        for i in range(self.num_obs):
            a = -8
            b = 8
            x = a + (b - a) * np.random.rand()
            y = a + (b - a) * np.random.rand()
            while abs(x - self._initpos.x) < 2:
                x = a + (b - a) * np.random.rand()
            while abs(y - self._initpos.y) < 2:
                y = a + (b - a) * np.random.rand()
            self.x_obs[i] = x
            self.y_obs[i] = y

        # Add obstacles to the environment
        for i in range(self.num_obs):
            obstacle_mat = chrono.ChMaterialSurfaceNSC()
            obstacle = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z,
                                                 1.0, 1.0,  # radius, height
                                                 100,       # density
                                                 True,      # visualization?
                                                 False,      # collision?
                                                 obstacle_mat)   # contact material(
            obstacle.SetPos(chrono.ChVectorD(
                self.x_obs[i], self.y_obs[i], 0.5))
            obstacle.SetBodyFixed(True)
            obstacle.GetVisualShape(0).SetTexture(
                chrono.GetChronoDataFile('textures/concrete.jpg'), 200, 200)
            self.system.Add(obstacle)

    # ------------------------------------- TODO: Add Sensors if necessary -------------------------------------

    def add_sensors(self):
        """
        Add sensors to the rover
        """

        pass

    # ------------ Check for collision with objects -------------------------------------
    def check_collision(self):
        """
        Check if we collide with any of the objects
        """
        collide = False
        cur_pos = self.rover.GetChassis().GetPos()
        for i in range(self.num_obs):
            if abs(cur_pos.x - self.x_obs[i]) < 1 and abs(cur_pos.y - self.y_obs[i]) < 1:
                collide = True
                break

        self._collision = collide
