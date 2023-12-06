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
# ========================================================================================================
# Authors: Huzaifa Unjhawala (refactored from original code by Simone Benatti, Aaron Young, Asher Elmquist)
# ========================================================================================================
import gymnasium as gym
import numpy as np
import math
import os
from gym_chrono.envs.utils.terrain_utils import SCMParameters
from gym_chrono.envs.utils.perlin_bitmap_generator import generate_random_bitmap
from gym_chrono.envs.utils.asset_utils import *
from gym_chrono.envs.utils.utils import CalcInitialPose, chVector_to_npArray, npArray_to_chVector, SetChronoDataDirectories
from gym_chrono.envs.ChronoBase import ChronoBaseEnv
import pychrono.vehicle as veh
import pychrono as chrono
from typing import Any

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


# Bunch of utilities required for the environment
# Standard Python imports

# Gymnasium imports


class off_road_gator(ChronoBaseEnv):

    # Supported render modes
    # Human - Render birds eye vier of the vehicle
    metadata = {'additional_render.modes': ['agent_pov', 'None']}

    def __init__(self, additional_render_mode='None'):
        # Check if render mode is suppoerted
        if additional_render_mode not in off_road_gator.metadata['additional_render.modes']:
            raise Exception(
                f'Render mode: {additional_render_mode} not supported')
        ChronoBaseEnv.__init__(self, additional_render_mode)

        # Ser the Chrono data directories for all the assest information
        SetChronoDataDirectories()

        # -------------------------------
        # Action and Observation Space
        # -------------------------------

        # Set camera frame as this is the observation
        self.m_camera_width = 80
        self.m_camera_height = 45

        # Observation space has 2 components
        # 1. Camera image (RGB) of size (m_camera_width, m_camera_height)
        # 2. Vehicle state relative to the goal of size (5,)
        self.observation_space = gym.spaces.Dict({
            "image": gym.spaces.Box(low=0, high=255, shape=(
                3, self.m_camera_height, self.m_camera_width), dtype=np.uint8),
            "data": gym.spaces.Box(low=-100, high=100, shape=(5,), dtype=np.float32)})

        # Action space is the steering, throttle and braking where
        # Steering is between -1 and 1
        # Throttle is between -1 and 1, negative is braking
        # This is done to aide training - part of recommende rl tips to have symmetric action space
        self.action_space = gym.spaces.Box(
            low=np.array([-1.0, -1.0]), high=np.array([1.0, 1.0]), shape=(2,), dtype=np.float32)
        # -------------------------------
        # Simulation specific class variables
        # -------------------------------
        self.m_assets = None  # List of assets in the simulation
        self.m_system = None  # Chrono system
        self.m_vehicle = None  # Vehicle set in reset method
        self.m_vehicle_pos = None  # Vehicle position
        self.m_driver = None  # Driver set in reset method
        self.m_driver_input = None  # Driver input set in reset method
        self.m_chassis = None  # Chassis body of the vehicle
        self.m_chassis_body = None  # Chassis body of the vehicle
        self.m_chassis_collision_box = None  # Chassis collision box of the vehicle
        self.m_proper_collision = True
        # Initial location and rotation of the vehicle
        self.m_initLoc = None
        self.m_initRot = None
        self.m_contact_force = None  # Contact force on the vehicle

        # Control and dynamics frequency
        self.m_control_frequency = 10  # Control frequency of the simulation
        self.m_step_size = 1e-3  # Step size of the simulation
        self.m_steps_per_control = round(
            1 / (self.m_step_size * self.m_control_frequency))

        self.m_steeringDelta = 0.05  # At max the steering can change by 0.05 in 0.1 seconds
        self.m_throttleDelta = 0.1
        self.m_brakingDelta = 0.1

        # Terrrain
        self.m_terrain = None  # Actual deformable terrain
        self.m_min_terrain_height = -5  # min terrain height
        self.m_max_terrain_height = 5  # max terrain height
        self.m_terrain_length = 80.0  # size in X direction
        self.m_terrain_width = 80.0  # size in Y direction
        self.m_assets = []
        self.m_positions = []
        # Sensor manager
        self.m_sens_manager = None  # Sensor manager for the simulation
        self.m_have_camera = False  # Flag to check if camera is present
        self.m_camera = None  # Camera sensor
        self.m_have_gps = False
        self.m_gps = None  # GPS sensor
        self.m_gps_origin = None  # GPS origin
        self.m_have_imu = False
        self.m_imu = None  # IMU sensor
        self.m_imu_origin = None  # IMU origin
        self.m_camera_frequency = 20
        self.m_gps_frequency = 10
        self.m_imu_frequency = 100

        # -------------------------------
        # Gym Env specific parameters
        # -------------------------------
        self.m_max_time = 20  # Max time for each episode
        self.m_reward = 0  # Reward for the episode
        self.m_debug_reward = 0  # Debug reward for the episode
        # Reward helpers
        self.m_action = None  # Action taken by the agent
        self.m_old_action = None  # Action taken by the agent at previous time step
        # Position of goal as numpy array
        self.m_goal = None
        # Distance to goal at previos time step -> To gauge "progress"
        self.m_vector_to_goal = None
        self.m_vector_to_goal_noNoise = None
        self.m_old_distance = None
        # Observation of the environment
        self.m_observation = None
        # Flag to determine if the environment has terminated -> In the event of timeOut or reach goal
        self.m_terminated = False
        # Flag to determine if the environment has truncated -> In the event of a crash
        self.m_truncated = False
        # Flag to check if the render setup has been done -> Some problem if rendering is setup in reset
        self.m_render_setup = False
        # Flag to count success while testing
        self.m_success = False
        # Flag to check if there is a plan to render or not
        self.m_play_mode = False
        self.m_additional_render_mode = additional_render_mode

    def reset(self, seed=None, options=None):
        """
        Reset the environment to its initial state -> Set up for standard gym API
        :param seed: Seed for the random number generator
        :param options: Options for the simulation (dictionary)
        """

        # -------------------------------
        # Reset Chrono system
        # -------------------------------
        self.m_system = chrono.ChSystemNSC()
        self.m_system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
        self.m_system.SetCollisionSystemType(
            chrono.ChCollisionSystem.Type_BULLET)

        # -------------------------------
        # Reset the terrain
        # -------------------------------
        self.m_isFlat = True
        terrain_delta = 0.05
        self.m_isRigid = True

        # texture_rand = random.randint(0, 3)

        texture_file_options = [
            "terrain/textures/grass.jpg", "terrain/textures/dirt.jpg", "terrain/textures/Gravel034_1K-JPG/Gravel034_1K_Color.jpg", "terrain/textures/concrete.jpg"]

        # texture_file = texture_file_options[random.randint(
        #     0, len(texture_file_options) - 1)]
        texture_file = texture_file_options[-1]
        if self.m_isRigid:
            self.m_terrain = veh.RigidTerrain(self.m_system)
            patch_mat = chrono.ChMaterialSurfaceNSC()
            patch_mat.SetFriction(0.9)
            patch_mat.SetRestitution(0.01)
            if (self.m_isFlat):
                patch = self.m_terrain.AddPatch(
                    patch_mat, chrono.CSYSNORM, self.m_terrain_length*1.5, self.m_terrain_width*1.5)
            else:
                # Initialize the terrain using a bitmap for the height map
                bitmap_file = os.path.dirname(os.path.realpath(
                    __file__)) + "/../data/terrain_bitmaps/height_map.bmp"

                # Some bitmap file backup (don't know why this is done in OG code)
                bitmap_file_backup = os.path.dirname(os.path.realpath(
                    __file__)) + "/../data/terrain_bitmaps/height_map_backup.bmp"

                generate_random_bitmap(shape=(252, 252), resolutions=[(2, 2)], mappings=[
                    (-1.5, 1.5)], file_name=bitmap_file)
                try:
                    patch = self.m_terrain.AddPatch(
                        patch_mat, chrono.CSYSNORM, bitmap_file, self.m_terrain_length*1.5, self.m_terrain_width*1.5, self.m_min_terrain_height, self.m_max_terrain_height)
                except:
                    print('Corrupt Bitmap File')
                    patch = self.m_terrain.AddPatch(
                        patch_mat, chrono.CSYSNORM, bitmap_file_backup, self.m_terrain_length*1.5, self.m_terrain_width*1.5, self.m_min_terrain_height, self.m_max_terrain_height)

            patch.SetTexture(veh.GetDataFile(
                texture_file), self.m_terrain_length*1.5, self.m_terrain_width*1.5)
            self.m_terrain.Initialize()

        else:
            # Real deformable terrain - Without mesh viz
            self.m_terrain = veh.SCMTerrain(self.m_system)
            # Set the SCM parameters
            terrain_params = SCMParameters()
            terrain_params.InitializeParametersAsMid()
            terrain_params.SetParameters(self.m_terrain)
            # Enable bulldozing effects
            self.m_terrain.EnableBulldozing(True)
            self.m_terrain.SetBulldozingParameters(
                55,  # angle of friction for erosion of displaced material at the border of the rut
                1,  # displaced material vs downward pressed material.
                5,  # number of erosion refinements per timestep
                10)  # number of concentric vertex selections subject to erosion
            self.m_terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, -0.5, 0), chrono.Q_from_AngX(
                -0)))

            if self.m_isFlat:
                # Initialize the terrain using a flat patch
                self.m_terrain.Initialize(
                    self.m_terrain_length * 1.5,  # Size in X direction
                    self.m_terrain_length * 1.5,  # Size in Y direction
                    terrain_delta)  # Mesh resolution
            else:
                # Initialize the terrain using a bitmap for the height map
                bitmap_file = os.path.dirname(os.path.realpath(
                    __file__)) + "/../data/terrain_bitmaps/height_map.bmp"

                # Some bitmap file backup (don't know why this is done in OG code)
                bitmap_file_backup = os.path.dirname(os.path.realpath(
                    __file__)) + "/../data/terrain_bitmaps/height_map_backup.bmp"

                generate_random_bitmap(shape=(252, 252), resolutions=[(2, 2)], mappings=[
                    (-1.5, 1.5)], file_name=bitmap_file)

                try:
                    self.m_terrain.Initialize(bitmap_file,  # heightmap file (.bmp)
                                              self.m_terrain_length * 1.5,  # sizeX
                                              self.m_terrain_width * 1.5,  # sizeY
                                              self.m_min_terrain_height,  # hMin
                                              self.m_max_terrain_height,  # hMax
                                              terrain_delta)  # mesh resolution
                except Exception:
                    print('Corrupt Bitmap File')
                    self.m_terrain.Initialize(bitmap_file_backup,  # heightmap file (.bmp)
                                              self.m_terrain_length * 1.5,  # sizeX
                                              self.m_terrain_width * 1.5,  # sizeY
                                              self.m_min_terrain_height,  # hMin
                                              self.m_max_terrain_height,  # hMax
                                              terrain_delta)

        # -------------------------------
        # Reset the vehicle
        # -------------------------------
        self.m_vehicle = veh.Gator(self.m_system)
        self.m_vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
        if (self.m_proper_collision):
            self.m_vehicle.SetChassisCollisionType(
                veh.CollisionType_PRIMITIVES)
        else:
            self.m_vehicle.SetChassisCollisionType(
                veh.CollisionType_NONE)
        self.m_vehicle.SetChassisFixed(False)
        # self.m_vehicle.SetTireType(veh.TireModelType_TMEASY)
        if (self.m_isRigid):
            self.m_vehicle.SetTireType(veh.TireModelType_TMEASY)
        else:
            self.m_vehicle.SetTireType(veh.TireModelType_RIGID_MESH)
        self.m_vehicle.SetTireStepSize(self.m_step_size)
        # Intialize position that depends on terrain
        gator_theta = self.initialize_gator_pos(seed)
        # Initialize the vehicle position -> get gator_theta to set the goal position
        self.m_vehicle.Initialize()

        # If we are visualizing, get mesh based viz
        if self.m_play_mode:
            self.m_vehicle.SetChassisVisualizationType(
                veh.VisualizationType_MESH)
            self.m_vehicle.SetWheelVisualizationType(
                veh.VisualizationType_MESH)
            self.m_vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)
        else:
            self.m_vehicle.SetChassisVisualizationType(
                veh.VisualizationType_PRIMITIVES)
            self.m_vehicle.SetWheelVisualizationType(
                veh.VisualizationType_PRIMITIVES)
            self.m_vehicle.SetTireVisualizationType(
                veh.VisualizationType_PRIMITIVES)
        self.m_vehicle.SetSuspensionVisualizationType(
            veh.VisualizationType_PRIMITIVES)
        self.m_vehicle.SetSteeringVisualizationType(
            veh.VisualizationType_PRIMITIVES)
        self.m_chassis_body = self.m_vehicle.GetChassisBody()

        # Set the driver
        self.m_driver = veh.ChDriver(self.m_vehicle.GetVehicle())
        self.m_driver_inputs = self.m_driver.GetInputs()

        # ===============================
        # Add the moving terrrain patches
        # ===============================
        if (self.m_isRigid == False):
            self.m_terrain.AddMovingPatch(self.m_chassis_body, chrono.ChVectorD(
                0, 0, 0), chrono.ChVectorD(5, 3, 1))
            # Set a texture for the terrain
            self.m_terrain.SetTexture(veh.GetDataFile(
                texture_file), self.m_terrain_length*2, self.m_terrain_width*2)

            # Set some vis
            self.m_terrain.SetPlotType(
                veh.SCMTerrain.PLOT_PRESSURE, 0, 30000.2)

        # -------------------------------
        # Set the goal point
        # -------------------------------
        self.set_goal(gator_theta, seed)

        # -------------------------------
        # Reset the obstacles
        # -------------------------------
        self.add_obstacles(proper_collision=False)

        # -------------------------------
        # Initialize the sensors
        # -------------------------------
        del self.m_sens_manager
        self.m_sens_manager = sens.ChSensorManager(self.m_system)
        # Set the lighting scene
        self.m_sens_manager.scene.AddPointLight(chrono.ChVectorF(
            100, 100, 100), chrono.ChColor(1, 1, 1), 5000.0)

        # Add all the sensors -> For now orientation is ground truth
        self.add_sensors(camera=True, gps=True, imu=False)

        # -------------------------------
        # Get the initial observation
        # -------------------------------
        self.m_observation = self.get_observation()
        self.m_old_distance = self.m_vector_to_goal.Length()
        self.m_old_action = np.zeros((2,))
        self.m_contact_force = 0
        self.m_debug_reward = 0
        self.m_reward = 0
        self.m_render_setup = False

        self.m_terminated = False
        self.m_truncated = False
        return self.m_observation, {}

    def step(self, action):
        """
        Gator takes a step in the environment - Frequency by default is 10 Hz
        """
        steering = action[0]
        # Negative throttle is braking
        if (action[1] < 0):
            throttle = 0
            braking = -action[1]
        else:
            throttle = action[1]
            braking = 0

        # This is used in the reward function
        self.m_action = action

        # smooth the driver inputs by preventing large changes
        self.m_driver_inputs.m_steering = np.clip(
            steering, self.m_driver_inputs.m_steering - self.m_steeringDelta, self.m_driver_inputs.m_steering + self.m_steeringDelta)
        self.m_driver_inputs.m_throttle = np.clip(
            throttle, self.m_driver_inputs.m_throttle - self.m_throttleDelta, self.m_driver_inputs.m_throttle + self.m_throttleDelta)
        self.m_driver_inputs.m_braking = np.clip(
            braking, self.m_driver_inputs.m_braking - self.m_brakingDelta, self.m_driver_inputs.m_braking + self.m_brakingDelta)

        # forward dynamics (since control happens at 10 Hz but dynamics at a higher frenquency same input is applied for 0.1 seconds)
        for i in range(0, self.m_steps_per_control):
            time = self.m_system.GetChTime()

            self.m_terrain.Synchronize(time)
            self.m_vehicle.Synchronize(
                time, self.m_driver_inputs, self.m_terrain)
            if (self.m_render_setup and self.render_mode == 'follow'):
                self.vis.Synchronize(time, self.m_driver_inputs)

            # Advance the vehicle
            self.m_driver.Advance(self.m_step_size)
            self.m_terrain.Advance(self.m_step_size)
            self.m_vehicle.Advance(self.m_step_size)
            if (self.m_render_setup and self.render_mode == 'follow'):
                self.vis.Advance(self.m_step_size)

            self.m_system.DoStepDynamics(self.m_step_size)
            # Sensor update
            self.m_sens_manager.Update()

            contact = self.m_assets.CheckContact(
                self.m_chassis_body, proper_collision=self.m_proper_collision)
            if contact:
                break
        # Get the observation
        self.m_observation = self.get_observation()
        self.m_reward = self.get_reward()
        self.m_debug_reward += self.m_reward

        # Check if we hit something or reached the goal
        self._is_terminated()
        self._is_truncated()

        return self.m_observation, self.m_reward, self.m_terminated, self.m_truncated, {}

    def render(self, mode='human'):
        """
        Render the environment
        """

        # ------------------------------------------------------
        # Add visualization - only if we want to see "human" POV
        # ------------------------------------------------------
        if mode == 'human':
            self.render_mode = 'human'

            if self.m_render_setup == False:
                self.vis = chronoirr.ChVisualSystemIrrlicht()
                self.vis.AttachSystem(self.m_system)
                self.vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
                self.vis.SetWindowSize(1280, 720)
                self.vis.SetWindowTitle('Gator in the wild')
                self.vis.Initialize()
                self.vis.AddSkyBox()
                self.vis.AddCamera(chrono.ChVectorD(
                    0, 0, 80), chrono.ChVectorD(0, 0, 1))
                self.vis.AddTypicalLights()
                self.vis.AddLightWithShadow(chrono.ChVectorD(
                    1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0.5), 3, 4, 10, 40, 512)
                self.m_render_setup = True

            self.vis.BeginScene()
            self.vis.Render()
            self.vis.EndScene()
        elif mode == 'follow':
            self.render_mode = 'follow'
            if self.m_render_setup == False:
                self.vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
                self.vis.SetWindowTitle('Gator in the wild')
                self.vis.SetWindowSize(1280, 1024)
                trackPoint = chrono.ChVectorD(0.0, 0.0, 1.75)
                self.vis.SetChaseCamera(trackPoint, 6.0, 0.5)
                self.vis.Initialize()
                self.vis.AddLightDirectional()
                self.vis.AddSkyBox()
                self.vis.AttachVehicle(self.m_vehicle.GetVehicle())
                self.m_render_setup = True

            self.vis.BeginScene()
            self.vis.Render()
            self.vis.EndScene()
        # else:
            # raise NotImplementedError

    def get_observation(self):
        """
        Get the observation of the environment
            1. Camera image (RGB) of size (m_camera_width, m_camera_height)
            2. Delta x of the goal in local frame of the vehicle
            3. Delta y of the goal in local frame of the vehicle
            4. Vehicle heading
            5. Heading needed to reach the goal
            6. Velocity of the vehicle     
        :return: Observation of the environment
        """
        self.m_vehicle_pos = self.m_chassis_body.GetPos()
        rgba = None
        # Get the camera observation
        if self.m_have_camera:
            camera_buffer_RGBA8 = self.m_camera.GetMostRecentRGBA8Buffer()
            if camera_buffer_RGBA8.HasData():
                rgba = camera_buffer_RGBA8.GetRGBA8Data()[:, :, 0:3]
            else:
                rgba = np.zeros((self.m_camera_height, self.m_camera_width, 3))
        else:
            # rgba = np.zeros((self.m_camera_height, self.m_camera_width, 3))
            raise Exception(
                'Camera not present - This demo needs camera setup')

        # Get GPS info
        cur_gps_data = None
        if self.m_have_gps:
            gps_buffer = self.m_gps.GetMostRecentGPSBuffer()
            if gps_buffer.HasData():
                cur_gps_data = gps_buffer.GetGPSData()
                cur_gps_data = chrono.ChVectorD(
                    cur_gps_data[1], cur_gps_data[0], cur_gps_data[2])
            else:
                cur_gps_data = chrono.ChVectorD(self.m_gps_origin)

            # Convert to cartesian coordinates
            sens.GPS2Cartesian(cur_gps_data, self.m_gps_origin)
        else:  # If there is no GPS use ground truth
            cur_gps_data = self.m_vehicle_pos

        if self.m_have_imu:
            raise NotImplementedError('IMU not implemented yet')

        self.m_vector_to_goal = self.m_goal - cur_gps_data
        self.m_vector_to_goal_noNoise = self.m_goal - self.m_vehicle_pos
        vector_to_goal_local = self.m_chassis_body.GetRot().RotateBack(self.m_vector_to_goal)

        vehicle_heading = self.m_chassis_body.GetRot().Q_to_Euler123().z
        vehicle_heading = self.m_vehicle.GetVehicle().GetRot().Q_to_Euler123().z

        target_heading_to_goal = np.arctan2(
            self.m_vector_to_goal.y, self.m_vector_to_goal.x)
        vehicle_speed = self.m_chassis_body.GetPos_dt().Length()
        observation_array = np.array(
            [vector_to_goal_local.x, vector_to_goal_local.y, vehicle_heading, target_heading_to_goal, vehicle_speed]).astype(np.float32)

        # Flip rgba to (3, height, width) from (height, width, 3)
        rgba = np.transpose(rgba, (2, 0, 1)).astype(np.uint8)
        obs_dict = {"image": rgba, "data": observation_array}
        return obs_dict

    def get_reward(self):
        """
        Not using delta action for now
        """
        # Compute the progress made
        progress_scale = 20.  # coefficient for scaling progress reward
        distance = self.m_vector_to_goal_noNoise.Length()
        # The progress made with the last action
        progress = self.m_old_distance - distance

        reward = progress_scale * progress

        # If we have not moved even by 1 cm in 0.1 seconds give a penalty
        if np.abs(progress) < 0.01:
            reward -= 10

        self.m_old_distance = distance

        return reward

    def _is_terminated(self):
        """
        Check if the environment is terminated
        """
        # If we are within a certain distance of the goal -> Terminate and give big reward
        # if np.linalg.norm(self.observation[:3] - self.goal) < 0.4:
        if np.linalg.norm(self.m_vector_to_goal_noNoise.Length()) < 10:
            print('--------------------------------------------------------------')
            print('Goal Reached')
            print('Initial position: ', self.m_initLoc)
            print('Goal position: ', self.m_goal)
            print('--------------------------------------------------------------')
            self.m_reward += 2500
            self.m_debug_reward += self.m_reward
            self.m_terminated = True
            self.m_success = True

        # If we have exceeded the max time -> Terminate and give penalty for how far we are from the goal
        if self.m_system.GetChTime() > self.m_max_time:
            print('--------------------------------------------------------------')
            print('Time out')
            print('Initial position: ', self.m_initLoc)
            # dist = np.linalg.norm(self.observation[:3] - self.goal)
            dist = self.m_vector_to_goal_noNoise.Length()
            print('Final position of Gator: ',
                  self.m_chassis_body.GetPos())
            print('Goal position: ', self.m_goal)
            print('Distance to goal: ', dist)
            # Give it a reward based on how close it reached the goal
            # self.m_reward -= 400
            self.m_reward -= 10 * dist

            self.m_debug_reward += self.m_reward
            print('Reward: ', self.m_reward)
            print('Accumulated Reward: ', self.m_debug_reward)
            print('--------------------------------------------------------------')
            self.m_terminated = True

    def _is_truncated(self):
        """
        Check if we have crashed or fallen off terrain
        """
        collision = self.m_assets.CheckContact(
            self.m_chassis_body, proper_collision=self.m_proper_collision)
        if collision:
            self.m_reward -= 600
            print('--------------------------------------------------------------')
            print(f'Crashed')
            print('--------------------------------------------------------------')
            self.m_debug_reward += self.m_reward
            self.m_truncated = True
        if (self._fallen_off_terrain()):
            self.m_reward -= 600
            print('--------------------------------------------------------------')
            print('Fallen off terrain')
            print('--------------------------------------------------------------')
            self.m_debug_reward += self.m_reward
            self.m_truncated = True

    def _fallen_off_terrain(self):
        """
        Check if we have fallen off the terrain
        For now just checks if the CG of the vehicle is within the rectangle bounds with some tolerance
        """
        terrain_length_tolerance = self.m_terrain_length
        terrain_width_tolerance = self.m_terrain_width

        vehicle_is_outside_terrain = abs(self.m_vehicle_pos.x) > terrain_length_tolerance or abs(
            self.m_vehicle_pos.y) > terrain_width_tolerance
        if (vehicle_is_outside_terrain):
            return True
        else:
            return False

    def initialize_gator_pos(self, seed):
        """
        Initialize the robot position
        :param seed: Seed for the random number generator
        :return: Random angle between 0 and 2pi along which gator is oriented
         """
        # Random angle between 0 and 2pi
        theta = random.random() * 2 * np.pi
        x, y = self.m_terrain_length * 0.5 * \
            np.cos(theta), self.m_terrain_width * 0.5 * np.sin(theta)
        z = self.m_terrain.GetHeight(chrono.ChVectorD(x, y, 0.)) + 0.25
        ang = np.pi + theta
        self.m_initLoc = chrono.ChVectorD(x, y, z)
        self.m_initRot = chrono.Q_from_AngZ(ang)
        self.m_vehicle.SetInitPosition(
            chrono.ChCoordsysD(self.m_initLoc, self.m_initRot))
        return theta

    def set_goal(self, gator_theta, seed):
        """
        Set the goal point
        :param seed: Seed for the random number generator
        """
        # Random angle between -pi/2 and pi/2
        delta_theta = (random.random() - 0.5) * 1.0 * np.pi
        # ensure that the goal is always an angle between -pi/2 and pi/2 from the gator
        gx, gy = self.m_terrain_length * 0.5 * np.cos(gator_theta + np.pi + delta_theta), self.m_terrain_width * 0.5 * np.sin(
            gator_theta + np.pi + delta_theta)
        self.m_goal = chrono.ChVectorD(
            gx, gy, self.m_terrain.GetHeight(chrono.ChVectorD(gx, gy, 0)) + 1.0)

        # Modify the goal point to be minimum 15 m away from gator
        i = 0
        while (self.m_goal - self.m_initLoc).Length() < 15:
            gx = random.random() * self.m_terrain_length - self.m_terrain_length / 2
            gy = random.random() * self.m_terrain_width - self.m_terrain_width / 2
            self.m_goal = chrono.ChVectorD(
                gx, gy, self.m_max_terrain_height + 1)
            if i > 100:
                print('Failed setting goal randomly, using default')
                gx = self.m_terrain_length * 0.625 * \
                    np.cos(gator_theta + np.pi + delta_theta)
                gy = self.m_terrain_width * 0.625 * \
                    np.sin(gator_theta + np.pi + delta_theta)
                break
            i += 1

        # Set the goal visualization
        goal_contact_material = chrono.ChMaterialSurfaceNSC()
        goal_mat = chrono.ChVisualMaterial()
        goal_mat.SetAmbientColor(chrono.ChColor(1., 0., 0.))
        goal_mat.SetDiffuseColor(chrono.ChColor(1., 0., 0.))

        goal_body = chrono.ChBodyEasySphere(
            0.55, 1000, True, False, goal_contact_material)

        goal_body.SetPos(self.m_goal)
        goal_body.SetBodyFixed(True)
        goal_body.GetVisualShape(0).SetMaterial(0, goal_mat)

        self.m_system.Add(goal_body)

    def add_obstacles(self, proper_collision=False):
        """Add obstacles to the terrain using asset utilities"""
        self.m_proper_collision = proper_collision

        if (self.m_proper_collision):
            # Create baseline type of rock assets
            rock1 = Asset(visual_shape_path="sensor/offroad/rock1.obj",
                          scale=1, bounding_box=chrono.ChVectorD(3.18344, 3.62827, 0))
            rock2 = Asset(visual_shape_path="sensor/offroad/rock2.obj",
                          scale=1, bounding_box=chrono.ChVectorD(4.01152, 2.64947, 0))
            rock3 = Asset(visual_shape_path="sensor/offroad/rock3.obj",
                          scale=1, bounding_box=chrono.ChVectorD(2.53149, 2.48862, 0))
            rock4 = Asset(visual_shape_path="sensor/offroad/rock4.obj",
                          scale=1, bounding_box=chrono.ChVectorD(2.4181, 4.47276, 0))
            rock5 = Asset(visual_shape_path="sensor/offroad/rock5.obj",
                          scale=1, bounding_box=chrono.ChVectorD(3.80205, 2.56996, 0))
        else:  # If there is no proper collision then collision just based on distance
            # Create baseline type of rock assets
            rock1 = Asset(visual_shape_path="sensor/offroad/rock1.obj",
                          scale=1)
            rock2 = Asset(visual_shape_path="sensor/offroad/rock2.obj",
                          scale=1)
            rock3 = Asset(visual_shape_path="sensor/offroad/rock3.obj",
                          scale=1)
            rock4 = Asset(visual_shape_path="sensor/offroad/rock4.obj",
                          scale=1)
            rock5 = Asset(visual_shape_path="sensor/offroad/rock5.obj",
                          scale=1)

        # Add these Assets to the simulationAssets
        self.m_assets = SimulationAssets(
            self.m_system, self.m_terrain, self.m_terrain_length, self.m_terrain_width)

        rock1_random = random.randint(0, 10)
        rock2_random = random.randint(0, 10)
        rock3_random = random.randint(0, 10)

        self.m_assets.AddAsset(rock1, number=rock1_random)
        self.m_assets.AddAsset(rock2, number=rock2_random)
        self.m_assets.AddAsset(rock3, number=rock3_random)
        # self.m_assets.AddAsset(rock4, number=2)
        # self.m_assets.AddAsset(rock5, number=2)

        # Randomly position these assets and add them to the simulation
        self.m_assets.RandomlyPositionAssets(self.m_goal, self.m_chassis_body)

    def add_sensors(self, camera=True, gps=True, imu=True):
        """
        Add sensors to the simulation
        :param camera: Flag to add camera sensor
        :param gps: Flag to add gps sensor
        :param imu: Flag to add imu sensor
        """
        # -------------------------------
        # Add camera sensor
        # -------------------------------
        if camera:
            self.m_have_camera = True
            cam_loc = chrono.ChVectorD(0.65, 0, 0.75)
            cam_rot = chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))
            cam_frame = chrono.ChFrameD(cam_loc, cam_rot)

            self.m_camera = sens.ChCameraSensor(
                self.m_chassis_body,  # body camera is attached to
                self.m_camera_frequency,  # update rate in Hz
                cam_frame,  # offset pose
                self.m_camera_width,  # image width
                self.m_camera_height,  # image height
                chrono.CH_C_PI / 3,  # FOV
                # supersampling factor (higher improves quality of the image)
                6
            )
            self.m_camera.SetName("Camera Sensor")
            self.m_camera.PushFilter(sens.ChFilterRGBA8Access())
            if (self.m_additional_render_mode == 'agent_pov'):
                self.m_camera.PushFilter(sens.ChFilterVisualize(
                    self.m_camera_width, self.m_camera_height, "Agent POV"))
            self.m_sens_manager.AddSensor(self.m_camera)
        if gps:
            self.m_have_gps = True
            std = 0.01  # GPS noise standard deviation - Good RTK GPS
            gps_noise = sens.ChNoiseNormal(chrono.ChVectorD(
                0, 0, 0), chrono.ChVectorD(std, std, std))
            gps_loc = chrono.ChVectorD(0, 0, 0)
            gps_rot = chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))
            gps_frame = chrono.ChFrameD(gps_loc, gps_rot)
            self.m_gps_origin = chrono.ChVectorD(43.073268, -89.400636, 260.0)

            self.m_gps = sens.ChGPSSensor(
                self.m_chassis_body,
                self.m_gps_frequency,
                gps_frame,
                self.m_gps_origin,
                gps_noise
            )
            self.m_gps.SetName("GPS Sensor")
            self.m_gps.PushFilter(sens.ChFilterGPSAccess())
            self.m_sens_manager.AddSensor(self.m_gps)
        if imu:
            self.m_have_imu = True
            std = 0.01
            imu_noise = sens.ChNoiseNormal(chrono.ChVectorD(
                0, 0, 0), chrono.ChVectorD(std, std, std))
            imu_loc = chrono.ChVectorD(0, 0, 0)
            imu_rot = chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))
            imu_frame = chrono.ChFrameD(imu_loc, imu_rot)
            self.m_imu_origin = chrono.ChVectorD(43.073268, -89.400636, 260.0)
            self.m_imu = sens.ChIMUSensor(
                self.m_chassis_body,
                self.m_imu_frequency,
                imu_frame,
                imu_noise,
                self.m_imu_origin
            )
            self.m_imu.SetName("IMU Sensor")
            self.m_imu.PushFilter(sens.ChFilterMagnetAccess())
            self.m_sens_manager.AddSensor(self.m_imu)

    def set_nice_vehicle_mesh(self):
        self.m_play_mode = True

    def close(self):
        del self.m_vehicle
        del self.m_sens_manager
        del self.m_system
        del self.m_assets.system
        del self.m_assets
        del self

    def __del__(self):
        del self.m_sens_manager
        del self.m_system
        del self.m_assets.system
        del self.m_assets
