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
# Authors: Json Zhou
# =======================================================================================
#
# This file contains a gym environment for the cobra rover on a flat terrain performing
# trajectory tracking. 
#
# =======================================================================================
#
# Action Space: The action space is the steering [-1,1]
#
# =======================================================================================
#
# Observation Space: The observation space contains 21 numbers, index 0 is the motor speed,
# which is given by defualt, index 1-10 contains past 10 observed heading errors, index 11-20
# contains past 10 action history
# Note that these history information is needed for smoothness requirement for the action
# =======================================================================================


# Chrono imports
from gymnasium.core import Env
import pychrono as chrono
import pychrono.robot as robot_chrono
import cmath
import random
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

class cobra_wpts(ChronoBaseEnv):
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

        # Define action space -> Now only steering
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(1,), dtype=np.float64)

        # Current lower and upper bounds
        lower_bounds = np.array([-np.pi]*11)
        upper_bounds = np.array([np.pi]*11)

        # Extend with the new bounds for the action history (-1.0 to 1.0 for each of the 10 actions)
        lower_bounds = np.concatenate((lower_bounds, np.array([-1.0]*10)))
        upper_bounds = np.concatenate((upper_bounds, np.array([1.0]*10)))

        # Update the observation space with the new bounds
        self.observation_space = gym.spaces.Box(
            low=lower_bounds,
            high=upper_bounds,
            dtype=np.float64)

        # -----------------------------
        # Chrono simulation parameters
        # -----------------------------
        self.system = None  # Chrono system set in reset method
        self.ground = None  # Ground body set in reset method
        self.rover = None  # Rover set in reset method

        self._initpos = chrono.ChVectorD(
            0.0, 0.0, 0.3)  # Rover initial position
        # Frequncy in which we apply control
        self._control_frequency = 5
        # Dynamics timestep
        self._step_size = 1e-3
        # Number of steps dynamics has to take before we apply control
        self._steps_per_control = round(
            1 / (self._step_size * self._control_frequency))
        self._collision = False
        self._terrain_length = 60
        self._terrain_width = 60
        self._terrain_height = 0.4
        self.rover_pos = None

        # ---------------------------------
        # Gym Environment variables
        # ---------------------------------
        # Maximum simulation time (seconds)
        self._max_time = 180
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

        self.reset()
    
        

    def reset(self, seed=None, options=None):
        """
        Reset the environment to its initial state -> Set up for standard gym API
        :param seed: Seed for the random number generator
        :param options: Options for the simulation (dictionary)
        """
        # Smoothness Reward
        self.action_history = []
        self.action_history_size = 10 

        # Heading error history register
        self.heading_error_history = []
        self.heading_error_history_size = 10

        # Cobra tracking lookahead
        self.lookahead = 2.0

        # Get the directory of the current file
        dir_path = os.path.dirname(os.path.realpath(__file__))

        # Path files used for training
        path_files = [
            os.path.join(dir_path, "../data/environment/room_1/PATH1.txt"),
            os.path.join(dir_path, "../data/environment/room_1/PATH2.txt"),
            os.path.join(dir_path, "../data/environment/room_1/PATH3.txt"),
            os.path.join(dir_path, "../data/environment/room_1/PATH4.txt"),
            os.path.join(dir_path, "../data/environment/room_1/PATH5.txt"),
        ]
        self.path_file_name = random.choice(path_files)
        
        # Waypoints
        self.x_coords, self.y_coords, self.z_coords = [], [], []

        # Read waypoints from file
        def read_waypoints(file_path):
            x_coords, y_coords, z_coords = [], [], []
            try:
                with open(file_path, 'r') as file:
                    for line in file:
                        x, y, z = line.strip().split(' ')
                        x_coords.append(float(x))
                        y_coords.append(float(y))
                        z_coords.append(float(z))
                return x_coords, y_coords, z_coords
            except FileNotFoundError:
                raise FileNotFoundError("File not found")
            except Exception as e:
                raise Exception("An error occurred") from e
            
        self.x_coords, self.y_coords, self.z_coords = read_waypoints(self.path_file_name)



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
        self.motor_driver_speed = random.uniform(chrono.CH_C_PI / 6, chrono.CH_C_PI / 2)
        self.driver = robot_chrono.CobraSpeedDriver(
            1/self._control_frequency, self.motor_driver_speed)
        self.rover.SetDriver(self.driver)

        # Choose two random waypoints from the path to initialize Cobra
        waypoints_indices = random.sample(range(len(self.x_coords)), 2)
        point1 = (self.x_coords[waypoints_indices[0]], self.y_coords[waypoints_indices[0]])
        point2 = (self.x_coords[waypoints_indices[1]], self.y_coords[waypoints_indices[1]])

        # Calculate the expected orientation
        orientation = random.uniform(-np.pi, np.pi)

        # Initialize position of the robot at one of the points
        self._initpos = chrono.ChVectorD(point1[0], point1[1], 0.4) # Assuming a fixed z-coordinate

        self._initrot = chrono.ChQuaternionD(1,0,0,0)
        self._initrot.Q_from_AngZ(orientation)

        self.rover.Initialize(chrono.ChFrameD(
            self._initpos, self._initrot))

        self.cur_pos = self._initpos
        
        # -----------------------------
        # Visualize way points
        # -----------------------------
        # -----------------------------
        # Set up goal visualization
        # -----------------------------
        for i in range(len(self.x_coords)):
            goal_contact_material = chrono.ChMaterialSurfaceNSC()
            goal_mat = chrono.ChVisualMaterial()
            goal_mat.SetAmbientColor(chrono.ChColor(1., 0., 0.))
            goal_mat.SetDiffuseColor(chrono.ChColor(1., 0., 0.))

            goal_body = chrono.ChBodyEasySphere(
                0.1, 1000, True, False, goal_contact_material)

            goal_body.SetPos(chrono.ChVectorD(
                self.x_coords[i], self.y_coords[i], 0.2))
            goal_body.SetBodyFixed(True)
            goal_body.GetVisualShape(0).SetMaterial(0, goal_mat)

            self.system.Add(goal_body)

        # -----------------------------
        # Get the intial observation
        # -----------------------------
        self.update_headingerror()
        self.observation = self.get_observation()

        self._debug_reward = 0

        self._terminated = False
        self._truncated = False
    

        return self.observation, {}

    def step(self, action):
        """
        Take a step in the environment - Frequency by default is 10 Hz.
        """
        # Record into the smoothness
        self.action_history.append(action)
        if len(self.action_history) > self.action_history_size:
            # Keep only the last 'action_history_size' actions
            self.action_history.pop(0)

        # Linearly interpolate steer angle between pi/6 and pi/8
        steer_angle = action[0] * self.max_steer
        self.driver.SetSteering(steer_angle)  # Maybe we should ramp this steer

        for i in range(self._steps_per_control):
            self.rover.Update()
            self.system.DoStepDynamics(self._step_size)
            self.update_headingerror()

        # Add the current heading error to the history
        self.heading_error_history.append(self.heading_error)
        if len(self.heading_error_history) > self.heading_error_history_size:
            # Keep only the last 'heading_error_history_size' heading errors
            self.heading_error_history.pop(0)

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
                    0, 5, 5), self.cur_pos)
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
        
        # Reward in this env contains three conponents:
        # 1. Soothness Penalty
        # 2. Path Proximity Reweard
        # 3. Deviation Penalty
        reward = 0

        # Smoothness Penalty
        smoothness_penalty = self.calculate_smoothness_penalty()
        reward = reward - smoothness_penalty

        # Path Proximity Reward
        path_proximity_reward = self.calculate_path_proximity_reward()
        reward = reward + path_proximity_reward

        # Deviation Penalty
        path_deviation_penalty = self.calculate_path_deviation_penalty()
        reward = reward - path_deviation_penalty

        # Debug Output
        # print("smooth reward:")
        # print(smoothness_penalty)
        # print("proxy reward:")
        # print(path_proximity_reward)
        # print("dev reward:")
        # print(path_deviation_penalty)
        # print("tot reward: ")
        # print(self.reward)

        return reward

    def _is_terminated(self):
        """
        Check if the environment is terminated
        """
        # If we have exceeded the max time -> Terminate
        if self.system.GetChTime() > self._max_time:
            print('--------------------------------------------------------------')
            print('Time out')
            print('Accumulated Reward: ', self._debug_reward)
            print('--------------------------------------------------------------')
            self._terminated = True
        else:
            self._terminated = False

    def _is_truncated(self):
        """
        Check if the environment is truncated
        """

        # If Cobra deviates the path by 3m, we truncate the simulation, no need to proceed
        min_distance = float('inf')
        for x, y, z in zip(self.x_coords, self.y_coords, self.z_coords):
            distance = self.euclidean_distance(self.rover_pos.x, self.rover_pos.y, x, y)
            if distance < min_distance:
                min_distance = distance

        # If it happens, penalize the agent
        if min_distance > 3:
            self._truncated = True
            self.reward -= 5000
            self._debug_reward -=5000
        else:
            self._truncated = False

    def get_observation(self):

        # Initialize observation
        observation = np.zeros(21)

        # Index 0: motor driver speed
        observation[0] = self.motor_driver_speed

        # Index 1 - 10: heading error with past 10 history
        observation[1] = self.heading_error
        for i in range(2, 11):
            if i-2 < len(self.heading_error_history):
                observation[i] = self.heading_error_history[i-2]
            else:
                observation[i] = observation[1]  # If not enough history, pad with the first observations

        # Index 10 - 20: action with past 10 history - for smoothness requirement
        for i in range(11, 11+self.action_history_size):
            if i-11 < len(self.action_history):
                observation[i] = self.action_history[i-11][0]
            else:
                observation[i] = 0.0

        return observation

    def euclidean_distance(self, x1, y1, x2, y2):
        # Helper function
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def update_headingerror(self):
        # Helper function
        self.rover_pos = self.rover.GetChassis().GetPos()
        self.rover_rot = self.rover.GetChassis().GetRot()
        self.rover_rot_euler = chrono.Q_to_Euler123(self.rover_rot)
            
        yaw = self.rover_rot_euler.z
           
        forward_x = math.cos(yaw)
        forward_y = math.sin(yaw)

        x_ahead = self.rover_pos.x + self.lookahead  * forward_x
        y_ahead = self.rover_pos.y + self.lookahead *  forward_y
            
        min_distance = float('inf')
        closest_waypoint = None

        for x, y, z in zip(self.x_coords, self.y_coords, self.z_coords):
            distance = self.euclidean_distance(x, y, x_ahead, y_ahead)
            if distance < min_distance:
                min_distance = distance
                closest_waypoint = (x, y, z)
            
        desired_heading = math.atan2(
            closest_waypoint[1] - y_ahead,
            closest_waypoint[0] - x_ahead
        )
            
        # Calculate heading error
        self.heading_error = desired_heading - yaw

        # Normalize the error to the range [-pi, pi]
        self.heading_error = (self.heading_error + math.pi) % (2 * math.pi) - math.pi
            

    def calculate_smoothness_penalty(self):
        # Helper function
        if len(self.action_history) < self.action_history_size:
            return 0

        # Extract scalar values from each array in the action history
        scalar_actions = [action[0] for action in self.action_history]

        # Convert the scalar action list to a NumPy array
        actions = np.array(scalar_actions)

        # Calculate the standard deviation
        action_std = np.std(actions)

        # Reward is higher for lower standard deviation
        smoothness_scaling_factor = 2.4
        smoothness_penalty = action_std * smoothness_scaling_factor

        return smoothness_penalty

    def calculate_path_proximity_reward(self):
        # Helper function
        min_distance = float('inf')
        for x, y, z in zip(self.x_coords, self.y_coords, self.z_coords):
            distance = self.euclidean_distance(self.rover_pos.x, self.rover_pos.y, x, y)
            if distance < min_distance:
                min_distance = distance

        # Reward inversely proportional to distance, with a maximum cutoff
        max_proximity_reward = 20  # Adjust as needed
        proximity_reward = max_proximity_reward / (1 + min_distance)
        return proximity_reward

    def calculate_path_deviation_penalty(self):
        # Helper function
        # Find the closest waypoint
        min_distance = float('inf')
        for x, y, z in zip(self.x_coords, self.y_coords, self.z_coords):
            distance = self.euclidean_distance(self.rover_pos.x, self.rover_pos.y, x, y)
            if distance < min_distance:
                min_distance = distance

        # Penalty for deviation larger than 1 meter
        deviation_threshold = 1.0  # 1 meter threshold
        penalty = 0
        penalty_scaling_factor = 10.0
        if min_distance > deviation_threshold:
            penalty = (min_distance - deviation_threshold) * penalty_scaling_factor  # Define an appropriate scaling factor

        return penalty