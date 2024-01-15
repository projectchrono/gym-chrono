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
# Authors: Huzaifa Unjhawala
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


class off_road_art(ChronoBaseEnv):

    # Supported render modes
    # Human - Render birds eye vier of the vehicle
    metadata = {'additional_render.modes': ['agent_pov', 'None']}

    def __init__(self, additional_render_mode='None'):
        # Check if render mode is suppoerted
        if additional_render_mode not in off_road_art.metadata['additional_render.modes']:
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
        # 1. 2D lidar scan of size (180,)
        # 2. Vehicle state relative to the goal of size (5,)
        self.observation_space = gym.spaces.Dict({
            "lidar": gym.spaces.Box(low=0, high=30, shape=(90,), dtype=np.float32),
            "data": gym.spaces.Box(low=-100, high=100, shape=(4,), dtype=np.float32)})

        # Action space is the steering, throttle and braking where
        # Steering is between -1 and 1
        # Throttle is between -1 and 1, negative is braking
        # This is done to aide training - part of recommende rl tips to have symmetric action space
        self.action_space = gym.spaces.Box(
            low=np.array([ -1.0]), high=np.array([1.0]), shape=(1,), dtype=np.float32)
        # -------------------------------
        # Simulation specific class variables
        # -------------------------------
        self.m_assets = None  # List of assets in the simulation
        self.m_system = None  # Chrono system
        self.m_vehicle = None  # Vehicle set in reset method
        self.m_old_vehcile_pos = None
        self.m_vehicle_pos = None  # Vehicle position
        self.m_driver = None  # Driver set in reset method
        self.m_driver_input = None  # Driver input set in reset method
        self.m_chassis = None  # Chassis body of the vehicle
        self.m_chassis_body = None  # Chassis body of the vehicle
        self.m_chassis_collision_box = None  # Chassis collision box of the vehicle
        self.m_proper_collision = False
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
        self.m_min_terrain_height = -1.5  # min terrain height
        self.m_max_terrain_height = 1.5  # max terrain height
        self.m_terrain_length = 100.0  # size in X direction
        self.m_terrain_width = 100.0  # size in Y direction
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
        self.m_have_lidar = False
        self.m_lidar = None  # Lidar sensor
        self.m_lidar_max_dist = 0
        self.m_lidar_horizontal_samples = 0
        self.m_camera_frequency = 20
        self.m_gps_frequency = 10
        self.m_imu_frequency = 100
        self.m_lidar_frequency = 10

        # -------------------------------
        # Gym Env specific parameters
        # -------------------------------
        self.m_max_time = 40  # Max time for each episode
        self.m_reward = 0  # Reward for the episode
        self.m_debug_reward = 0  # Debug reward for the episode
        # Reward helpers
        self.m_action = None  # Action taken by the agent
        self.m_old_action = None  # Action taken by the agent at previous time step
        # Position of goal as numpy array
        self.m_goal_vis = True  # Set true to visualize the goal
        self.m_error_state = None
        self.m_old_error_state = None
        self.m_old_position = None
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

        self.m_episode_num = 0
        self.m_success_count = 0
        self.m_crash_count = 0
        self.m_fallen_count = 0
        self.m_timeout_count = 0
        self.m_success_rate_eval = 0.

        self.m_mean_obstacles = 5  # Starting mean is 1 - at checkpoint 43, its 5
        self.m_std_dev = 2         # Standard deviation
        self.info = {"is_success": False}

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

        texture_file_options = [
            "terrain/textures/grass.jpg", "terrain/textures/dirt.jpg", "terrain/textures/Gravel034_1K-JPG/Gravel034_1K_Color.jpg", "terrain/textures/concrete.jpg"]

        # texture_file = texture_file_options[random.randint(
        #     0, len(texture_file_options) - 1)]
        texture_file = texture_file_options[0]
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
            terrain_params.InitializeParametersAsHard()
            terrain_params.SetParameters(self.m_terrain)
            # Enable bulldozing effects
            self.m_terrain.EnableBulldozing(False)
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

        # Intialize the collision system -> only then can we query the height of the terrain in the rigid case
        if (self.m_isRigid):
            self.m_system.GetCollisionSystem().Initialize()
        # -------------------------------
        # Reset the vehicle
        # -------------------------------

        self.m_vehicle = veh.ARTcar(self.m_system)
        self.m_vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
        self.m_vehicle.SetChassisCollisionType(
            veh.CollisionType_NONE)
        self.m_vehicle.SetChassisFixed(False)
        if (self.m_isRigid):
            self.m_vehicle.SetTireType(veh.TireModelType_TMEASY)
        else:
            self.m_vehicle.SetTireType(veh.TireModelType_RIGID_MESH)
        self.m_vehicle.SetTireStepSize(self.m_step_size)
        # Intialize position that depends on terrain
        art_theta, x, y = self.initialize_art_pos(seed)
        self.m_vehicle.SetTireRollingResistance(0.06)
        self.m_vehicle.SetMaxMotorVoltageRatio(0.18)
        self.m_vehicle.SetStallTorque(0.5)
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
        # Set the path for the vehicle
        # -------------------------------
        self.set_path(self.m_terrain_length*0.5, self.m_terrain_width*0.5, seed)

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
        self.add_sensors(lidar=True, camera=False, gps=True, imu=False)

        # -------------------------------
        # Get the initial observation
        # -------------------------------
        self.m_observation = self.get_observation()
        self.m_old_action = np.zeros((1,))
        self.m_contact_force = 0
        self.m_debug_reward = 0
        self.m_reward = 0
        self.m_render_setup = False

        # Success count for eval
        self.m_success_count_eval = 0
        print("Mean obstacle at reset ", self.m_mean_obstacles)
        self.m_terminated = False
        self.m_truncated = False
        self.info["is_success"] = False
        return self.m_observation, self.info

    def step(self, action):
        """
        art takes a step in the environment - Frequency by default is 10 Hz
        """
        steering = action[0]
        # Negative throttle is braking
        # if (action[1] < 0):
        #     throttle = 0
        #     braking = -action[1]
        # else:
        #     throttle = action[1]
        #     braking = 0
        throttle = 0.3
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
        #TODO: Add reward.
        self.m_reward = self.get_reward()
        self.m_debug_reward += self.m_reward

        # Check if we hit something or reached the goal
        self._is_terminated()
        self._is_truncated()

        return self.m_observation, self.m_reward, self.m_terminated, self.m_truncated, self.info

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
                self.vis.SetWindowTitle('art in the wild')
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
                self.vis.SetWindowTitle('art in the wild')
                self.vis.SetWindowSize(1280, 1024)
                trackPoint = chrono.ChVectorD(2, 0.0, 0.1)
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
            1. Lidar scan of (90,)
            2. Delta x of the goal in local frame of the vehicle
            3. Delta y of the goal in local frame of the vehicle
            4. Vehicle heading
            5. Heading needed to reach the goal
            6. Velocity of the vehicle     
        :return: Observation of the environment
        """

        # Get lidar scan
        if self.m_have_lidar:
            lidar_buffer = self.m_lidar.GetMostRecentDIBuffer()
            if lidar_buffer.HasData():
                lidar_data = lidar_buffer.GetDIData()
                lidar_data = np.array(lidar_data)
                # get only depth from (1*self.m_lidar_horizontal_samples*2)
                lidar_data = lidar_data[:, :, 0]
                # flatten to 180,
                lidar_data = lidar_data.reshape(
                    (self.m_lidar_horizontal_samples,))
                # Set all 0 values to max range to signify "full depth"
                lidar_data[lidar_data == 0] = self.m_lidar_max_dist
                # Do the min reduce to get self.m_lidar_horizontal_samples/chunk features
                reduce_chunk = 2
                lidar_data = [min(lidar_data[i:i+reduce_chunk])
                              for i in range(0, len(lidar_data), reduce_chunk)]
                lidar_data = np.array(lidar_data, dtype=np.float32)
                # clip values above 30
                lidar_data = np.clip(lidar_data, 0, 30)
            else:
                lidar_data = np.zeros((90,), dtype=np.float32)
            for i in range(len(lidar_data)):
                lidar_data[i] = 30#random.uniform(0, 30)
        else:
            raise Exception(
                'Lidar not present - This demo needs camera setup')
        self.m_old_vehicle_pos = self.m_vehicle_pos
        self.m_vehicle_pos = self.m_chassis_body.GetPos()
        rgba = None
        # Get the camera observation
        if self.m_have_camera:
            camera_buffer_RGBA8 = self.m_camera.GetMostRecentRGBA8Buffer()
            if camera_buffer_RGBA8.HasData():
                rgba = camera_buffer_RGBA8.GetRGBA8Data()[:, :, 0:3]
            else:
                rgba = np.zeros((self.m_camera_height, self.m_camera_width, 3))

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

        #we want to get the closest point on the path to the vehicle, and then get the error state between ourselves and that point.
        x_current = self.m_vehicle_pos.x
        y_current = self.m_vehicle_pos.y
        theta_current = self.m_chassis_body.GetRot().Q_to_Euler123().z
        v_current = self.m_chassis_body.GetPos_dt().Length()
        dist = np.zeros((1, len(self.m_x_path)))
        for i in range(0, len(self.m_x_path)):
            dist[0][i] = (
                x_current + np.cos(theta_current) * 1 - self.m_x_path[i]
            ) ** 2 + (
                y_current + np.sin(theta_current) * 1 - self.m_y_path[i]
            ) ** 2
        index = dist.argmin()
        err_theta = 0
        ref_state_current = [self.m_x_path[index], self.m_y_path[index], self.m_theta_path[index], 1]
        
        ref = ref_state_current[2]
        act = theta_current

        if (ref > 0 and act > 0) or (ref <= 0 and act <= 0):
            err_theta = ref - act
        elif ref <= 0 and act > 0:
            if abs(ref - act) < abs(2 * np.pi + ref - act):
                err_theta = -abs(act - ref)
            else:
                err_theta = abs(2 * np.pi + ref - act)
        else:
            if abs(ref - act) < abs(2 * np.pi - ref + act):
                err_theta = abs(act - ref)
            else:
                err_theta = -abs(2 * np.pi - ref + act)

        RotM = np.array(
            [
                [np.cos(-theta_current), -np.sin(-theta_current)],
                [np.sin(-theta_current), np.cos(-theta_current)],
            ]
        )

        errM = np.array(
            [[ref_state_current[0] - x_current], [ref_state_current[1] - y_current]]
        )

        errRM = RotM @ errM

        observation_array = [
            errRM[0][0],
            errRM[1][0],
            err_theta,
            ref_state_current[3] - v_current,
        ]




        self.m_old_error_state = self.m_error_state

        self.m_error_state = observation_array
        obs_dict = {
            "lidar": lidar_data, 
            "data": observation_array}
        return obs_dict

    def euclidean_distance(self, x1, y1, x2, y2):
        # Helper function
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def get_reward(self):
    # Reward in this env contains three conponents:
        # 1. Soothness Penalty
        # 2. Path Proximity Reweard
        # 3. Deviation Penalty
        reward = 0

        # Smoothness Penalty
        # steering_penalty = self.calculate_steering_penalty()
        # reward = reward - steering_penalty

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
    
    def calculate_path_deviation_penalty(self):
        # Helper function
        # Find the closest waypoint
        min_distance = float('inf')
        for x, y in zip(self.m_x_path, self.m_y_path):
            distance = self.euclidean_distance(self.m_vehicle_pos.x, self.m_vehicle_pos.y, x, y)
            if distance < min_distance:
                min_distance = distance

        # Penalty for deviation larger than 1 meter
        deviation_threshold = 1.0  # 1 meter threshold
        penalty = 0
        penalty_scaling_factor = 10.0
        if min_distance > deviation_threshold:
            penalty = (min_distance - deviation_threshold) * penalty_scaling_factor  # Define an appropriate scaling factor

        return penalty

    def calculate_steering_penalty(self):

        if(abs(self.m_error_state[2])>np.pi/2):
            return 1
        else:
            return 0
        
    def calculate_path_proximity_reward(self):
        # Helper function
        min_distance = float('inf')
        for x, y in zip(self.m_x_path, self.m_y_path):
            distance = self.euclidean_distance(self.m_vehicle_pos.x, self.m_vehicle_pos.y, x, y)
            if distance < min_distance:
                min_distance = distance

        # Reward inversely proportional to distance, with a maximum cutoff
        max_proximity_reward = 1.0  # Adjust as needed
        proximity_reward = max_proximity_reward / (1 + min_distance)
        if proximity_reward > 0.5:
            proximity_reward += self.calculate_heading_reward()
        return proximity_reward

    def calculate_heading_reward(self):
        max_heading_reward = 1.0
        return max_heading_reward/(1+self.m_error_state[2])
    def _is_terminated(self):
        """
        Check if the environment is terminated
        """
        # If we have exceeded the max time -> Terminate and give penalty for how far we are from the goal
        if self.m_system.GetChTime() > self.m_max_time:
            print('--------------------------------------------------------------')
            print('Time out')
            print('Initial position: ', self.m_initLoc)
            # dist = np.linalg.norm(self.observation[:3] - self.goal)
            print('Final position of art: ',
                  self.m_chassis_body.GetPos())

            self.m_debug_reward += self.m_reward
            print('Reward: ', self.m_reward)
            print('Accumulated Reward: ', self.m_debug_reward)
            print('--------------------------------------------------------------')
            self.m_terminated = True
            self.m_episode_num += 1
            self.m_timeout_count += 1

    def _is_truncated(self):
        """
        Check if we have crashed or fallen off terrain
        """
        collision = self.m_assets.CheckContact(
            self.m_chassis_body, proper_collision=self.m_proper_collision)
        if self.euclidean_distance(self.m_error_state[0],self.m_error_state[1],0,0) > 5:
            self.m_reward -=30
            self.m_debug_reward += self.m_reward
            print('--------------------------------------------------------------')
            print(f'Deviated from path')
            print('Accumulated Reward: ', self.m_debug_reward)
            print('--------------------------------------------------------------')
            self.m_truncated = True
            self.m_episode_num += 1
            self.m_crash_count += 1
        if collision:
            #self.m_reward -= 50
            self.m_debug_reward += self.m_reward
            print('--------------------------------------------------------------')
            print(f'Crashed')
            print('Accumulated Reward: ', self.m_debug_reward)
            print('--------------------------------------------------------------')

            self.m_truncated = True
            self.m_episode_num += 1
            self.m_crash_count += 1
        if (self._fallen_off_terrain()):
            #self.m_reward -= 30
            self.m_debug_reward += self.m_reward
            print('--------------------------------------------------------------')
            print('Fallen off terrain')
            print('Accumulated Reward: ', self.m_debug_reward)
            print('--------------------------------------------------------------')
            self.m_truncated = True
            self.m_episode_num += 1
            self.m_fallen_count += 1

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

    def initialize_art_pos(self, seed):
        """
        Initialize the robot position
        :param seed: Seed for the random number generator
        :return: Random angle between 0 and 2pi along which art is oriented
         """
        # Random angle between 0 and 2pi
        theta = 0#random.random() * 2 * np.pi
        x, y = self.m_terrain_length * 0.5, self.m_terrain_width * 0.5
        z = self.m_terrain.GetHeight(chrono.ChVectorD(x, y, 100)) + 0.25
        ang = theta
        self.m_initLoc = chrono.ChVectorD(x, y, z)
        self.m_initRot = chrono.Q_from_AngZ(ang)
        self.m_vehicle.SetInitPosition(
            chrono.ChCoordsysD(self.m_initLoc, self.m_initRot))
        return theta,x,y

    def set_path(self, off_x, off_y, seed=None):   
        H = 10
        rho = np.random.rand(H) * np.logspace(-0.5, -2.5, H)
        phi = np.random.rand(H) * 2 * np.pi

        # Accumulate r(t) over t=[0,2*pi]
        t = np.linspace(0, 2 * np.pi, 200)
        r = np.ones_like(t)
        #multiply all value in r by 10
        for h in range(H):
            r = r + rho[h] * np.sin((h + 1) * t + phi[h])

        # Reconstruct x(t), y(t)
        x = r * np.cos(t)
        y = r * np.sin(t)
        x = x*10
        y = y*10
        # use atan2 to get the slope, dydx, between the current poitn and the previous point.
        #first, use the difference between the first point and the last point.
        #then, use the difference between the current point and the previous point.
        dydx = []
        num = []
        dydx.append(np.arctan2(y[-1] - y[0], x[-1] - x[0]))
        num.append(0)
        for i in range(1, len(x)):
            dydx.append(np.arctan2(y[i] - y[i-1], x[i] - x[i-1]))
            num.append(i)

        #move the numbers dydx, x, and y around the array as if it were a cicular buffer so that the first point has dydx as close to 0 as possible
        #find the index of the smallest value in dydx
        min = 100
        index = 0
        for i in range(len(dydx)):
            if(abs(dydx[i]) < min):
                min = abs(dydx[i])
                index = i
        #move the first index to the index of the smallest value
        dydx = np.roll(dydx, -index)
        x = np.roll(x, -index)
        y = np.roll(y, -index)
        # add offset to x and y
        self.m_x_path = x + self.m_initLoc.x-x[0]
        self.m_y_path = y + self.m_initLoc.y-y[0]
        self.m_theta_path = dydx
        # Set the goal visualization
        if (self.m_goal_vis):
            for i in range(len(self.m_x_path)):
                m_goal = chrono.ChVectorD(self.m_x_path[i], self.m_y_path[i], 0.1)
                goal_contact_material = chrono.ChMaterialSurfaceNSC()
                goal_mat = chrono.ChVisualMaterial()
                goal_mat.SetAmbientColor(chrono.ChColor(1., 0., 0.))
                goal_mat.SetDiffuseColor(chrono.ChColor(1., 0., 0.))

                goal_body = chrono.ChBodyEasySphere(
                    0.1, 1000, True, False, goal_contact_material)
                
                goal_body.SetPos(m_goal)
                goal_body.SetBodyFixed(True)
                goal_body.GetVisualShape(0).SetMaterial(0, goal_mat)

                self.m_system.Add(goal_body)


    def update_mean_based_on_success(self):
        if self.m_success_rate_eval > 0.7:
            if (self.m_mean_obstacles < 5):
                self.m_mean_obstacles += 1  # Increase mean if success rate is high
                print("Updated Number of obstacles mean to ",
                      self.m_mean_obstacles)
                print("Success rate is ", self.m_success_rate_eval)
            else:
                print("Max mean of obstacles = 5 reached")
                print("Success rate is ", self.m_success_rate_eval)
            self.m_success_rate_eval = 0.0   # Reset success rate after adjustment

    def set_succ(self, succ):
        self.m_success_rate_eval = succ

    def set_mean_obs(self, mean_obs):
        self.m_mean_obstacles = mean_obs

    def add_obstacles(self, proper_collision=False):
        """Add obstacles to the terrain using asset utilities"""
        self.m_proper_collision = proper_collision
        self.update_mean_based_on_success()
        scale = 0.5
        if (self.m_proper_collision):
            # Create baseline type of rock assets
            rock1 = Asset(visual_shape_path="sensor/offroad/rock1.obj",
                          scale=scale, bounding_box=chrono.ChVectorD(3.18344, 3.62827, 0))
            rock2 = Asset(visual_shape_path="sensor/offroad/rock2.obj",
                          scale=scale, bounding_box=chrono.ChVectorD(4.01152, 2.64947, 0))
            rock3 = Asset(visual_shape_path="sensor/offroad/rock3.obj",
                          scale=scale, bounding_box=chrono.ChVectorD(2.53149, 2.48862, 0))
            rock4 = Asset(visual_shape_path="sensor/offroad/rock4.obj",
                          scale=scale, bounding_box=chrono.ChVectorD(2.4181, 4.47276, 0))
            rock5 = Asset(visual_shape_path="sensor/offroad/rock5.obj",
                          scale=scale, bounding_box=chrono.ChVectorD(3.80205, 2.56996, 0))
        else:  # If there is no proper collision then collision just based on distance
            # Create baseline type of rock assets
            rock1 = Asset(visual_shape_path="sensor/offroad/rock1.obj",
                          scale=scale)
            rock2 = Asset(visual_shape_path="sensor/offroad/rock2.obj",
                          scale=scale)
            rock3 = Asset(visual_shape_path="sensor/offroad/rock3.obj",
                          scale=scale)
            rock4 = Asset(visual_shape_path="sensor/offroad/rock4.obj",
                          scale=scale)
            rock5 = Asset(visual_shape_path="sensor/offroad/rock5.obj",
                          scale=scale)

        # Add these Assets to the simulationAssets
        self.m_assets = SimulationAssets(
            self.m_system, self.m_terrain, self.m_terrain_length, self.m_terrain_width)

        rock1_random = 0#max(
            # 0, min(7, round(random.gauss(self.m_mean_obstacles, self.m_std_dev))))
        rock2_random = 0#max(
            # 0, min(7, round(random.gauss(self.m_mean_obstacles, self.m_std_dev))))
        rock3_random = 0#max(
            # 0, min(6, round(random.gauss(self.m_mean_obstacles, self.m_std_dev))))
        # rock1_random = random.randint(0, 10)
        # rock2_random = random.randint(0, 10)
        # rock3_random = random.randint(0, 10)

        self.m_num_obstacles = rock1_random + rock2_random + rock3_random

        self.m_assets.AddAsset(rock1, number=rock1_random)
        self.m_assets.AddAsset(rock2, number=rock2_random)
        self.m_assets.AddAsset(rock3, number=rock3_random)
        # self.m_assets.AddAsset(rock4, number=2)
        # self.m_assets.AddAsset(rock5, number=2)

    def add_sensors(self, lidar=True,  camera=True, gps=True, imu=True):
        """
        Add sensors to the simulation
        :param lidar: Flag to add lidar sensor
        :param camera: Flag to add camera sensor
        :param gps: Flag to add gps sensor
        :param imu: Flag to add imu sensor
        """
        # -------------------------------
        # Add lidar sensor
        # -------------------------------
        if lidar:
            self.m_have_lidar = True
            lidar_loc = chrono.ChVectorD(0.1, 0, 0.08)
            lidar_rot = chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))
            lidar_frame = chrono.ChFrameD(lidar_loc, lidar_rot)
            self.m_lidar_max_dist = 30
            self.m_lidar_horizontal_samples = 180
            self.m_lidar = sens.ChLidarSensor(
                self.m_chassis_body,  # body lidar is attached to
                self.m_lidar_frequency,  # scanning rate in Hz
                lidar_frame,  # offset pose
                self.m_lidar_horizontal_samples,  # number of horizontal samples
                1,  # number of vertical channels
                chrono.CH_C_PI,  # horizontal field of view
                0.,
                0.,  # vertical field of view
                self.m_lidar_max_dist  # max distance
            )
            self.m_lidar.SetName("Lidar Sensor")
            self.m_lidar.SetLag(0.)
            self.m_lidar.SetCollectionWindow(0.)

            self.m_lidar.PushFilter(sens.ChFilterDIAccess())
            if (self.m_additional_render_mode == 'agent_pov'):
                self.m_lidar.PushFilter(
                    sens.ChFilterVisualize(640, 480, "2D Lidar"))
            self.m_sens_manager.AddSensor(self.m_lidar)

        # -------------------------------
        # Add camera sensor
        # -------------------------------
        if camera:
            self.m_have_camera = True
            cam_loc = chrono.ChVectorD(0.1, 0, 0.08)
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
