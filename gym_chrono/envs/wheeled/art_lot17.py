# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2021 projectchrono.org
# All right reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =====================================================================================
# Authors: Huzaifa Unjhawala
# =====================================================================================
#
# This environment is a gym environment to train the ART vehicle Chrono
# simulation to reach a point in the lot 17 parking lot
#
# ======================================================================================
#
# Action Space: The action space is the steering and throttle of the vehicle
# The steering is normalized between -1 (left turn) and 1
# The throttle is normalized between 0 and 1
# Box(low=np.array([-1.0, 0]), high=np.array([1.0, 1.0]), shape=(2,), dtype=np.float64)
#
# ======================================================================================
#
# Observation Space: The observation space consists of the following quantities
# 1. Delta x of the goal in local frame of the vehicle
# 2. Delta y of the goal in local frame of the vehicle
# 3. Vehicle heading
# 4. Heading needed to reach the goal
# 5. Velocity of vehicle
# 6. GPS Position transformed to Cartesian - X axis
# 7. GPS Position transformed to Cartesian - Y axis
# Box(low=-200, high=200, shape=(7,), dtype=np.float64)
#
# ======================================================================================


# Chrono imports
import pychrono as chrono
import pychrono.vehicle as veh
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


class art_lot17(ChronoBaseEnv):
    """
    Gym environment for the ART vehicle Chrono simulation to reach a point in the lot 17 parking lot
    """

    def __init__(self, render_mode=None):
        ChronoBaseEnv.__init__(self, render_mode)

        SetChronoDataDirectories()

        # Action space is the throttle and steering - Throttle is between 0 and 1, steering is -1 to 1
        self.action_space = gym.spaces.Box(
            low=np.array([-1.0, 0]), high=np.array([1.0, 1.0]), shape=(2,), dtype=np.float64)

        # Define observation space
        # First few elements describe the relative position of the rover to the goal
        # Delta x in local frame
        # Delta y in local frame
        # Vehicle heading
        # Heading needed to reach the goal
        # Velocity of vehicle
        # GPS Position transformed to Cartesian - X axis
        # GPS Position transformed to Cartesian - Y axis
        self._num_observations = 7
        self.observation_space = gym.spaces.Box(
            low=-200, high=200, shape=(self._num_observations,), dtype=np.float64)

        # -----------------------------
        # Chrono simulation parameters
        # -----------------------------
        self.system = None  # Chrono system set in reset method
        self.vehicle = None  # Vehicle set in reset method
        self.ground = None  # Ground body set in reset method
        self.art = None  # ART set in reset method
        self.driver = None  # Driver set in reset method
        self.driver_input = None  # Driver input set in reset method

        self.x_obs = None
        self.y_obs = None
        self.render_mode = render_mode
        self._initpos = chrono.ChVectorD(
            0.0, 0.0, 0.0)  # ART initial position

        # Frequncy in which we apply control
        self._control_frequency = 10
        # Dynamics timestep
        self._step_size = 1e-3
        # Number of steps dynamics has to take before we apply control
        self._steps_per_control = round(
            1 / (self._step_size * self._control_frequency))
        self._collision = False
        self.vehicle_pos = None

        self.sensor_manager = None
        self._have_gps = False  # Flag to check if GPS sensor is present
        self.gps = None  # GPS if needed is added in add_sensors
        self._have_imu = False  # Flag to check if IMU sensor is present
        self.gps_origin = None  # GPS origin in lat, long, alt
        self.goal_gps = None  # Goal in GPS frame
        self._sensor_frequency = 10  # Frequency of sensor frame update

        # -----------------------------
        # Terrain helper variables
        # -----------------------------
        self._terrain_length = 40
        self._terrain_width = 80
        self._terrain_height = 2
        self._terrain_center = chrono.ChVectorD(0, 0, 0)

        # If the vehicle is in contact with the wall, that is a collision
        self._wall_center = chrono.ChVectorD(0, 0, 0)
        self._wall_box_length = 20
        self._wall_box_width = 60

        # ---------------------------------
        # Gym Environment variables
        # ---------------------------------
        # Maximum simulation time (seconds)
        self._max_time = 100
        # Holds reward of the episode
        self.reward = 0
        self._debug_reward = 0
        # Position of goal as numpy array
        self.goal = None
        # Distance to goal at previos time step -> To gauge "progress"
        self._vector_to_goal = None
        self._vector_to_goal_withoutNoise = None
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
        # Initialize the vehicle
        self.vehicle = veh.ARTcar()

        # -----------------------------
        # Contact mand collision properties
        # -----------------------------
        contact_method = chrono.ChContactMethod_SMC
        self.vehicle.SetContactMethod(contact_method)
        self.vehicle.SetChassisCollisionType(False)  # No collision for now

        # ---------------------------------
        # Initailize positon
        # ----------------------------------
        self.initialize_vehicle_pos(seed)

        # -----------------------------
        # Set vehicle properties
        # -----------------------------
        self.vehicle.SetChassisFixed(False)
        self.vehicle.SetTireType(veh.TireModelType_TMEASY)
        self.vehicle.SetTireStepSize(self._step_size)
        self.vehicle.SetMaxMotorVoltageRatio(0.08)
        self.vehicle.SetStallTorque(0.3)
        self.vehicle.SetTireRollingResistance(0.015)
        self.vehicle.Initialize()

        # ------------------
        # Visualizations
        # ------------------
        self.vehicle.SetChassisVisualizationType(
            veh.VisualizationType_PRIMITIVES)
        self.vehicle.SetWheelVisualizationType(
            veh.VisualizationType_PRIMITIVES)
        self.vehicle.SetSuspensionVisualizationType(
            veh.VisualizationType_PRIMITIVES)
        self.vehicle.SetSteeringVisualizationType(
            veh.VisualizationType_PRIMITIVES)
        self.vehicle.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)
        self.chassis_body = self.vehicle.GetChassisBody()

        # ---------------------------
        # Get chrono system of the vehicle
        # ---------------------------
        self.system = self.vehicle.GetVehicle().GetSystem()
        self.system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

        # ---------------------------
        # Terrain
        # ---------------------------
        self.system = self.vehicle.GetVehicle().GetSystem()
        self.system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

        self.terrain = veh.RigidTerrain(self.system)
        patch_mat = chrono.ChMaterialSurfaceSMC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
        patch_mat.SetYoungModulus(2e7)

        # Just a flat terrain for now
        patch = self.terrain.AddPatch(
            patch_mat, chrono.CSYSNORM, self._terrain_length, self._terrain_width)
        patch.SetTexture(self.chronopath +
                         'textures/concrete.jpg', 200, 200)
        patch.SetColor(chrono.ChColor(1., 0., 0.))
        self.terrain.Initialize()

        # ---------------------------
        # Provide a delta to controls
        # ---------------------------
        # Set the time response for steering and throttle inputs.
        steering_time = 0.75
        # time to go from 0 to +1 (or from 0 to -1)
        throttle_time = .5
        # time to go from 0 to +1
        self.SteeringDelta = (self._step_size / steering_time)
        self.ThrottleDelta = (self._step_size / throttle_time)

        # ---------------------------
        # Add sensors
        # ---------------------------
        self.add_sensors()

        # ---------------------------
        # Get the driver system
        # ---------------------------
        self.driver = veh.ChDriver(self.vehicle.GetVehicle())
        self.driver_inputs = self.driver.GetInputs()

        # ---------------------------
        # Add obstacles - pass for now
        # ---------------------------
        self.add_obstacles(seed)

        # ---------------------------
        # Set up the goal
        # ---------------------------
        self.set_goalPoint(seed)

        # ---------------------------
        # Get the initial observation
        # ---------------------------
        self.observation = self.get_observation()
        self._old_distance = self._vector_to_goal_withoutNoise.Length()  # To track progress

        self._debug_reward = 0
        self._render_setup = False
        self._terminated = False
        self._truncated = False
        self._success = False  # During testing phase to see number of successes

        return self.observation, {}

    def step(self, action):
        """
        ART takes a step in the environment - Frequency by default is 10 Hz
        """
        steering = action[0]
        throttle = action[1]

        for i in range(self._steps_per_control):
            time = self.system.GetChTime()
            self.driver_inputs.m_steering = np.clip(steering, self.driver_inputs.m_steering - self.SteeringDelta,
                                                    self.driver_inputs.m_steering + self.SteeringDelta)
            self.driver_inputs.m_throttle = np.clip(throttle, self.driver_inputs.m_throttle - self.ThrottleDelta,
                                                    self.driver_inputs.m_throttle + self.ThrottleDelta)
            self.driver_inputs.m_braking = 0.0  # No braking for now

            self.vehicle.Synchronize(time, self.driver_inputs, self.terrain)
            self.terrain.Synchronize(time)

            # Advance the vehicle
            self.driver.Advance(self._step_size)
            self.vehicle.Advance(self._step_size)
            self.terrain.Advance(self._step_size)

            # Sensor update
            self.sensor_manager.Update()

        self.observation = self.get_observation()
        self.reward = self.get_reward()
        self._debug_reward += self.reward

        # Check if we hit something or reached the goal
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
                self.vis.SetWindowTitle('ART lot17 RL playground')
                self.vis.Initialize()
                self.vis.AddSkyBox()
                self.vis.AddCamera(chrono.ChVectorD(
                    0, 0, 32), chrono.ChVectorD(0, 0, 1))
                self.vis.AddTypicalLights()
                self.vis.AddLightWithShadow(chrono.ChVectorD(
                    1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0.5), 3, 4, 10, 40, 512)
                self._render_setup = True

            self.vis.BeginScene()
            self.vis.Render()
            self.vis.EndScene()
        elif mode == 'follow':
            if self._render_setup == False:
                self.vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
                self.vis.SetWindowTitle('ART')
                self.vis.SetWindowSize(1280, 1024)
                trackPoint = chrono.ChVectorD(0.0, 0.0, 1.75)
                self.vis.SetChaseCamera(trackPoint, 6.0, 0.5)
                self.vis.Initialize()
                self.vis.AddLightDirectional()
                self.vis.AddSkyBox()
                self.vis.AttachVehicle(self.vehicle.GetVehicle())
                self._render_setup = True

            self.vis.BeginScene()
            self.vis.Render()
            self.vis.EndScene()
        else:
            raise NotImplementedError

    def get_reward(self):
        """
        Get the reward for the current step. Reward  = 200 * progress_made.
        Progress made is the difference between the distance to the goal in the previous step and the current step
        progress made can be negative, in which case the reward is negative
        A fixed -10 reward for moving by less than a centimeter in 0.1 seconds
        :return: Reward for the current step
        """
        scale_pos = 20
        scale_neg = 20
        # Distance to goal
        # distance = np.linalg.norm(self.observation[:3] - self.goal)
        distance = self._vector_to_goal_withoutNoise.Length()  # chrono vector
        if (self._old_distance > distance):
            reward = scale_pos * (self._old_distance - distance)
        else:
            reward = scale_neg * (self._old_distance - distance)

        # If we have not moved even by 1 cm in 0.1 seconds, give penalty
        if (np.abs(self._old_distance - distance) < 0.01):
            reward -= 5

        # Update the old distance
        self._old_distance = distance

        return reward

    def _is_terminated(self):
        """
        Check if the environment is terminated
        If ART within 2 m of the goal - terminal +2500 reward
        If ART exceeded the max time - terminal -100 * distance to goal reward
        """
        # If we are within a certain distance of the goal -> Terminate and give big reward
        # if np.linalg.norm(self.observation[:3] - self.goal) < 0.4:
        if np.linalg.norm(self._vector_to_goal_withoutNoise.Length()) < 2:
            print('--------------------------------------------------------------')
            print('Goal Reached')
            print('Initial position: ', self._initpos)
            print('Goal position: ', self.goal)
            print('--------------------------------------------------------------')
            self.reward += 5000
            self._debug_reward += self.reward
            self._terminated = True
            self._success = True

        # If we have exceeded the max time -> Terminate
        if self.system.GetChTime() > self._max_time:
            print('--------------------------------------------------------------')
            print('Time out')
            print('Initial position: ', self._initpos)
            # dist = np.linalg.norm(self.observation[:3] - self.goal)
            dist = self._vector_to_goal_withoutNoise.Length()
            print('Final position of rover: ',
                  self.chassis_body.GetPos())
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
        Check if the episode is truncated due to collision with wall of falling off the terrain.
        Both handled seperately as of now if we want to give different rewards
        """
        # Check if we are in contact with the wall

        if (self._check_collision()):
            self.reward -= 300
            self._debug_reward += self.reward
            print('--------------------------------------------------------------')
            print('Collision')
            print('Vehicle Postion: ', self.vehicle_pos)
            print('Goal position: ', self.goal)
            print('Reward: ', self.reward)
            print('Accumulated Reward: ', self._debug_reward)
            print('--------------------------------------------------------------')
            self._truncated = True

        # Check if we have fallen off the terrain
        if (self._fallen_off_terrain()):
            self.reward -= 300
            self._debug_reward += self.reward
            print('--------------------------------------------------------------')
            print('Fallen off terrain')
            print('Vehicle Postion: ', self.vehicle_pos)
            print('Goal position: ', self.goal)
            print('Reward: ', self.reward)
            print('Accumulated Reward: ', self._debug_reward)
            print('--------------------------------------------------------------')
            self._truncated = True

    def get_observation(self):
        """
        Get the observation of the environment
        Position of vehicle from GPS
        Position of goal in cartesian
        Vehicle heading priveledged information
        Vehicle velocity priveledged information

        :return: Observation of the environment
                 1. Delta x in local frame
                 2. Delta y in local frame
                 3. Vehicle heading
                 4. Heading needed to reach the goal
                 5. Velocity of vehicle
                 6. GPS Position transformed to Cartesian - X axis
                 7. GPS Position transformed to Cartesian - Y axis
        """

        observation = np.zeros(self._num_observations)

        gps_buffer = self.gps.GetMostRecentGPSBuffer()
        cur_gps_data = None
        self.vehicle_pos = self.chassis_body.GetPos()
        # Get the GPS data from the buffer
        if self._have_gps:
            if gps_buffer.HasData():
                cur_gps_data = gps_buffer.GetGPSData()
                cur_gps_data = chrono.ChVectorD(
                    cur_gps_data[1], cur_gps_data[0], cur_gps_data[2])
            else:
                cur_gps_data = chrono.ChVectorD(self.gps_origin)
            # Position of vehicle in cartesian coodinates from the GPS buffer
            sens.GPS2Cartesian(cur_gps_data, self.gps_origin)
        else:  # There is no gps, use previledged information
            cur_gps_data = self.vehicle_pos

        # Goal is currently not read from the GPS sensor
        self._vector_to_goal = npArray_to_chVector(self.goal) - cur_gps_data
        # This is to calculate reward - there is no need for the reward to be noisy while training even if the observation is noisy
        self._vector_to_goal_withoutNoise = npArray_to_chVector(
            self.goal) - self.vehicle_pos
        self._vector_to_goal_withoutNoise.z = 0
        vector_to_goal_local = self.chassis_body.GetRot().RotateBack(self._vector_to_goal)
        self._vector_to_goal.z = 0  # Set this to zero to not effect reward calculation

        # TODO: Use magnetometer here to get heading - need help from Nevindu/Harry
        # For now using priveledged information
        vehicle_heading = self.chassis_body.GetRot().Q_to_Euler123().z
        # TODO: Use state estimator used in reality in simulation as well to get velocity - need help from Stefan/Ishaan
        # For now using priveldeged information
        vehicle_velocity = self.chassis_body.GetPos_dt()
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
        observation[5] = cur_gps_data.x
        observation[6] = cur_gps_data.y

        return observation

    def initialize_vehicle_pos(self, seed=1):
        """
        Initialize the pose of the robot
        The positon is randomly initialized between the four corners with a random noise of 1 m on x and y
        The rotation is a random angle between -pi and pi 
        """
        # Initialize vehicle at the left corner of the terrain

        # Choose Pick a corner
        corner = np.random.randint(0, 4)
        # rot_random = np.random.randint(0, 4)
        noise_x = np.random.uniform(low=0, high=1)
        noise_y = np.random.uniform(low=0, high=1)
        self._initRot = chrono.ChQuaternionD(1., 0, 0, 0)

        # rot_options = [-math.pi/2, math.pi/2, -math.pi, math.pi]

        x_mean = 15
        y_mean = 35
        if (corner == 0):
            self._initpos = chrono.ChVectorD(-x_mean +
                                             noise_x, y_mean + noise_y, 0.1)
            random_angle = np.random.uniform(low=-np.pi/2, high=0)
            self._initRot.Q_from_AngZ(random_angle)
        elif (corner == 1):
            self._initpos = chrono.ChVectorD(
                x_mean + noise_x, y_mean + noise_y, 0.1)
            random_angle = np.random.uniform(low=-np.pi, high=-np.pi/2)
            self._initRot.Q_from_AngZ(random_angle)
        elif (corner == 2):
            self._initpos = chrono.ChVectorD(-x_mean +
                                             noise_x, -y_mean + noise_y, 0.1)
            random_angle = np.random.uniform(low=0, high=np.pi/2.)
            self._initRot.Q_from_AngZ(random_angle)
        elif (corner == 3):
            self._initpos = chrono.ChVectorD(
                x_mean + noise_x, -y_mean + noise_y, 0.1)
            random_angle = np.random.uniform(low=np.pi/2, high=np.pi)
            self._initRot.Q_from_AngZ(random_angle)

        self.vehicle.SetInitPosition(
            chrono.ChCoordsysD(self._initpos, self._initRot))

        self.vehicle_pos = self._initpos

    def set_goalPoint(self, seed=None):
        """
        Set the goal point for the environment
        """
        # Set the goal point
        if seed is not None:
            np.random.seed(seed)

        # Select a random point in a rectange of dimension 32.5 X 65 centered at the origin
        # The point should not be within a rectangle of dimension 17.5 x 55 centered at the origin
        # The point should not be within 20 (vehicle_goal_dist_threshold) meters of where the vehicle starts

        wall_x_tolerance = 1.25
        wall_y_tolerance = 1.25

        goal_wall_x_max = self._wall_box_length/2 + wall_x_tolerance

        goal_wall_y_max = self._wall_box_width/2 + wall_y_tolerance

        boundary_x_tolerance = 2.5
        boundary_y_tolerance = 2.5

        goal_boundary_x_max = self._terrain_length/2 - boundary_x_tolerance

        goal_boundary_y_max = self._terrain_width/2 - boundary_y_tolerance

        vehicle_x_pos = self.vehicle_pos.x
        vehicle_y_pos = self.vehicle_pos.y

        vehicle_goal_dist_threshold = 20
        # Guess goal ensuring its within the boundaries
        self.goal = np.random.uniform(low=[-goal_boundary_x_max, -goal_boundary_y_max], high=[
            goal_boundary_x_max, goal_boundary_y_max], size=(2,))

        goal_is_inside_wall = (np.abs(self.goal[0]) < goal_wall_x_max) or (np.abs(
            self.goal[1]) < goal_wall_y_max)
        goal_is_close_to_vehicle = (math.sqrt(
            (self.goal[0] - vehicle_x_pos)**2 + (self.goal[1] - vehicle_y_pos)**2) < vehicle_goal_dist_threshold)

        while (goal_is_inside_wall or goal_is_close_to_vehicle):
            self.goal = np.random.uniform(low=[-self._terrain_length/2, -self._terrain_width/2], high=[
                self._terrain_length/2, self._terrain_width/2], size=(2,))

            goal_is_inside_wall = np.abs(self.goal[0]) < goal_wall_x_max and np.abs(
                self.goal[1]) < goal_wall_y_max

            goal_is_close_to_vehicle = (math.sqrt(
                (self.goal[0] - vehicle_x_pos)**2 + (self.goal[1] - vehicle_y_pos)**2) < vehicle_goal_dist_threshold)

        self.goal = np.append(self.goal, 0)

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
            self.goal[0], self.goal[1], 0.2))
        goal_body.SetBodyFixed(True)
        goal_body.GetVisualShape(0).SetMaterial(0, goal_mat)

        self.system.Add(goal_body)

    def _check_collision(self):
        """
        Check if we collided against the inner wall 
        Basically checks if the vehicle ground truth position is within the wall boundaries
        Just checking CG for now - no bounding box and no actual collision
        """
        vehicle_is_inside_walls = abs(
            self.vehicle_pos.x) < self._wall_box_length/2 and abs(self.vehicle_pos.y) < self._wall_box_width/2
        if (vehicle_is_inside_walls):
            return True
        else:
            return False

    def _fallen_off_terrain(self):
        """
        Check if we have fallen off the terrain
        For now just checks if the CG of the vehicle is within the rectangle bounds with some tolerance
        """
        terrain_length_tolerance = self._terrain_length/2
        terrain_width_tolerance = self._terrain_width/2

        vehicle_is_outside_terrain = abs(self.vehicle_pos.x) > terrain_length_tolerance or abs(
            self.vehicle_pos.y) > terrain_width_tolerance
        if (vehicle_is_outside_terrain):
            return True
        else:
            return False

    def add_sensors(self):
        """
        Add sensors to the vehicle
        """

        self._initialize_sensor_manager()
        self._add_gps_sensor(std=0.01)
        self._have_gps = True
        self._add_magnetometer_sensor(std=0)
        self._have_imu = True

    def _initialize_sensor_manager(self):
        """
        Initializes chrono sensor manager
        """
        self.sensor_manager = sens.ChSensorManager(self.system)
        self.sensor_manager.scene.AddPointLight(
            chrono.ChVectorF(100, 100, 100), chrono.ChColor(1, 1, 1), 5000)
        b = sens.Background()
        b.color_horizon = chrono.ChVectorF(.6, .7, .8)
        b.color_zenith = chrono.ChVectorF(.4, .5, .6)
        b.mode = sens.BackgroundMode_GRADIENT
        self.sensor_manager.scene.SetBackground(b)

    def _add_gps_sensor(self, std):
        """
        Add a GPS sensor to the vehicle
        :param std: Standard deviation of the GPS sensor
        """
        if (self.sensor_manager is None):
            self.initialize_sensor_manager()

        noise_model = sens.ChNoiseNormal(chrono.ChVectorD(
            0, 0, 0), chrono.ChVectorD(std, std, std))
        gps_offset_pose = chrono.ChFrameD(chrono.ChVectorD(
            0, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(1, 0, 0)))

        # The lat long and altitude of the cartesian origin - This needs to be measured
        self.gps_origin = chrono.ChVectorD(43.073268, -89.400636, 260.0)

        self.gps = sens.ChGPSSensor(self.vehicle.GetChassisBody(), self._sensor_frequency,  # update rate
                                    gps_offset_pose, self.gps_origin, noise_model)

        self.gps.SetName("GPS")
        self.gps.PushFilter(sens.ChFilterGPSAccess())
        self.sensor_manager.AddSensor(self.gps)

    def _add_magnetometer_sensor(self, std=0):
        """
        Add a magnetometer sensor to the vehicle
        :param std: Standard deviation of the magnetometer sensor
        """
        noise_model = sens.ChNoiseNormal(chrono.ChVectorD(
            0, 0, 0), chrono.ChVectorD(std, std, std))
        imu_offset_pose = chrono.ChFrameD(chrono.ChVectorD(
            0, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(1, 0, 0)))
        mag = sens.ChMagnetometerSensor(self.vehicle.GetChassisBody(
        ), 100, imu_offset_pose, noise_model, self.gps_origin)
        mag.SetName("IMU - Magnetometer")
        mag.PushFilter(sens.ChFilterMagnetAccess())
        self.sensor_manager.AddSensor(mag)

    def add_obstacles(self, seed):
        """
        Add obstacles to the environment
        """
        pass
