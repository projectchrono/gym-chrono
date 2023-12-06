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
# Authors: Huzaifa Unjhawala
# =======================================================================================
#
# This file contains the gym environment for the ART vehicle Chrono simulation to
# track way points that are specified in a csv file in ../data/training_wpts/ the name of
# the csv file is specified in the _wptFiles variable (needs to be set internally as of
# Nov 7 2023)
#
# =======================================================================================
#
# Action Space: The action space is the steering and throttle of the vehicle
# The steering is normalized between -1 (left turn) and 1
# The throttle is normalized between 0 and 1
# Box(low=np.array([-1.0, 0]), high=np.array([1.0, 1.0]), shape=(2,), dtype=np.float64)
#
# =======================================================================================
#
# Observastion Space: The observation space is the position and velocity of the 5 closest
# way points after applying the look ahead distance
# self.observation_space = gym.spaces.Box(
#           low=-100, high=100, shape=(self._num_wpts_observed*4,), dtype=np.float64)
#
# =======================================================================================

# pychrono imports
import pychrono as chrono
import pychrono.vehicle as veh
try:
    import pychrono.sensor as sens
except:
    print('Could not import Chrono Sensor')

import numpy as np
import math
import os
import random

# Custom imports
from gym_chrono.envs.ChronoBase import ChronoBaseEnv
from gym_chrono.envs.utils.utils import CalcInitialPose, chVector_to_npArray, SetChronoDataDirectories

# gym imports
import gymnasium as gym


class art_wpts(ChronoBaseEnv):
    """
    Gym environment for the ART vehicle Chrono simulation to track way points
    """

    def __init__(self, render_mode='human'):
        ChronoBaseEnv.__init__(self, render_mode)

        SetChronoDataDirectories()

        # Define action space
        # Action space is the throttle and steering - Throttle is between 0 and 1, steering is -1 to 1
        self.action_space = gym.spaces.Box(
            low=np.array([-1.0, 0]), high=np.array([1.0, 1.0]), shape=(2,), dtype=np.float64)

        # Define observation space -> This is the position and velocity of the 5 closest way points
        # Way points contain x,y,yaw and velocity

        # self._wptFiles = ['circ_r25_ccw.csv', 'circ_r25_cw.csv', 'circ_r2_ccw.csv',
        #                   'circ_r2_cw.csv', 'circ_r5_ccw.csv', 'circ_r5_cw.csv', 'circ_rinf.csv']
        self._wptFiles = ['circ_rinf.csv']

        self._num_wpts_observed = 5
        self.observation_space = gym.spaces.Box(
            low=-100, high=100, shape=(self._num_wpts_observed*4,), dtype=np.float64)

        # list of wpt files used for training

        # Step should take place at 10 Hz
        self._control_frequency = 10
        # Dynamics time step
        self._dyna_time_step = 2e-3

        # Max end time
        self._tend = 40

        self.play_mode = True
        self.terrain_length = 80.0  # size in X direction
        self.terrain_width = 80.0  # size in Y direction
        self.terrain_thickness = 5.0  # thickness in Z direction

        # Need a look ahead distance to specify which direction to mopve
        self._lookAheadDist = 1.
        self._reward_progress = 0
        self._reward_speed = 0
        self._reward_collision = 0
        self._reward = 0
        self._collision = False
        self._done = False
        self._terminated = False
        self._truncated = False
        self._isdone = False

        self.reward = 0

    def get_wpts(self, seed=10):
        """
        Read the way points from the csv file and set the initial position of the vehicle
        """
        # Pick a random int from 0 to 4 based on the seed
        # This is the file that will be used for training
        random.seed(seed)
        csv_file = self._wptFiles[random.randint(
            0, self._wptFiles.__len__()-1)]

        # Read the way points from the csv file
        self.wpts = np.genfromtxt(
            self.chronopath + '/training_wpts/' + csv_file, delimiter=',')[:150]

        # get x,y point at a particular index
        point1 = chrono.ChVectorD(self.wpts[0][0], self.wpts[0][1], 0)
        point2 = chrono.ChVectorD(self.wpts[1][0], self.wpts[1][1], 0)

        # Add noise to the initial point
        np.random.seed(seed)
        point1 = point1 + chrono.ChVectorD(np.random.normal(
            0, 0.01), np.random.normal(0, 0.01), 0)

        # Add noise to the second point
        point2 = point2 + chrono.ChVectorD(np.random.normal(
            0, 0.01), np.random.normal(0, 0.01), 0)

        self.initLoc, self.initRot = CalcInitialPose(point1, point2)

    def get_obs(self, point):
        """
        Get the observation which is the self._num_wpts_observed closest wpts. 
        IMPORTANT : 'point' is the point after applying lookahead distance
        """

        # Check if point is np.array
        if not isinstance(point, np.ndarray):
            raise TypeError
        # Get the distance of all the way points from the vehicle
        dist = np.linalg.norm(self.wpts[:, 0:2] - point, axis=1)

        # Get the index of the closest way point
        obs_index = np.argsort(dist)[:self._num_wpts_observed]

        # Get the observation in an array of size (self._num_wpts_observed, 3)
        observation = np.zeros((self._num_wpts_observed, 4))
        observation = self.wpts[obs_index, :]

        return observation

    def get_reward(self):

        progress_weight = 100
        progress_speed_weight = 1

        # Give a reward for making progress towards the closest way point from the lookahead
        closest_wpt = self.observation[0, :]
        # get the distance of the current positon of vehicle to closest wpt
        dist = np.linalg.norm(
            closest_wpt[0:2] - chVector_to_npArray(self.chassis_body.GetPos())[:2], axis=0)
        progress = self._old_dist - dist

        # Give a reward for progress in speed towards the speed of the closest way point
        speed = chVector_to_npArray(self.chassis_body.GetPos_dt())
        speed = np.linalg.norm(speed[0:2], axis=0)
        delta_speed = abs(closest_wpt[3] - speed)
        progress_speed = self._old_delta_speed - delta_speed

        speed_reward = 0
        # If delta speed is between -0.1 and 0.1 then give a nice reward
        if delta_speed > -0.1 and delta_speed < 0.1:
            speed_reward += 1

        # Punish standing still or low velocities
        if speed < 0.1:
            speed_reward += -1

        # If we improve the delta speed, that is the delta speed reduced give a reward as a factor of the improvement
        speed_reward += progress_speed * progress_speed_weight

        reward = progress_weight * progress + speed_reward

        ############## Debugging ################
        # print('----------------------------------')
        # print('Closest wpt:')
        # print('\t x: ', closest_wpt[0])
        # print('\t y: ', closest_wpt[1])

        # print('Current position:')
        # print('\t x: ', chVector_to_npArray(self.chassis_body.GetPos())[0])
        # print('\t y: ', chVector_to_npArray(self.chassis_body.GetPos())[1])

        # print('Distance to closest wpt: ', dist)
        # print('Old Distance ', self._old_dist)
        # print('Progress: ', progress)
        # print('dist progress reward:', progress_weight * progress)

        # print('Speed: ', speed)
        # print('Speed at closest wpt: ', closest_wpt[3])
        # print('Delta Speed: ', delta_speed)
        # print('Old Delta Speed: ', self._old_delta_speed)
        # print('Progress Speed: ', progress_speed)
        # print('Speed reward: ', speed_reward)

        self._old_dist = dist
        self._old_delta_speed = delta_speed

        return reward

    def is_terminated(self):
        """
        Terminate the epsiode if the goal state has been reached
        """
        # Check the last way point position
        last_wpt = self.wpts[-1, :]

        # Get the distance of the current positon of vehicle to last wpt
        dist = np.linalg.norm(
            last_wpt[0:2] - chVector_to_npArray(self.chassis_body.GetPos())[:2], axis=0)

        # If the distance is less than 1 m, terminate the episode
        if dist < 5:
            self._terminated = True
            self.reward += 1000

    def is_truncated(self):
        """
        Truncate episode if the closest way point is 50 m away from the vehicle or the time limit is reached
        """
        # If time limit reached
        if self.system.GetChTime() > self._tend:
            self._terminated = True
            print('Time limit reached')
            dist = np.linalg.norm(
                self.wpts[-1, 0:2] - chVector_to_npArray(self.chassis_body.GetPos())[:2], axis=0)
            print('Distance from the destination point')
            print(dist)
            print('\t x: ', self.wpts[-1, 0] -
                  chVector_to_npArray(self.chassis_body.GetPos())[0])
            print('\t y: ', self.wpts[-1, 1] -
                  chVector_to_npArray(self.chassis_body.GetPos())[1])

            # print where the vehicle is currently
            print('Current position:')
            print('\t x: ', chVector_to_npArray(self.chassis_body.GetPos())[0])
            print('\t y: ', chVector_to_npArray(self.chassis_body.GetPos())[1])

            # Scale negative reward by the distance from the destination
            penalty = -10 * dist
            self.reward += penalty

            print('Penalty:', penalty)
            print('Reward: ', self.reward)
        # If the closest way point is 50 m away from the vehicle
        closest_wpt = self.observation[0, :]
        dist = np.linalg.norm(
            closest_wpt[0:2] - chVector_to_npArray(self.chassis_body.GetPos())[:2], axis=0)
        if dist > 50:
            print('Too Far Away')
            print('Closest wpt:')
            print('\t x: ', closest_wpt[0])
            print('\t y: ', closest_wpt[1])

            print('Current position:')
            print('\t x: ', chVector_to_npArray(self.chassis_body.GetPos())[0])
            print('\t y: ', chVector_to_npArray(self.chassis_body.GetPos())[1])

            print('Distance: ', dist)

            self._truncated = True
            self.reward += -1000

    def reset(self, seed=None, options=None):
        self.get_wpts(seed)
        self.vehicle = veh.RCCar()

        # --------------------------------
        # Contact and collision properties
        # --------------------------------

        contact_method = chrono.ChContactMethod_SMC
        self.vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
        # self.vehicle.SetChassisCollisionType(
        #     veh.ChassisCollisionType_NONE)  # No collision for now
        self.vehicle.SetChassisCollisionType(False)  # No collision for now

        # ---------------------------------
        # Initailize positon and tire model
        # ----------------------------------
        self.vehicle.SetChassisFixed(False)
        self.vehicle.SetInitPosition(
            chrono.ChCoordsysD(self.initLoc, self.initRot))
        self.vehicle.SetTireType(veh.TireModelType_TMEASY)
        self.vehicle.SetTireStepSize(self._dyna_time_step)
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
        # patch = self.terrain.AddPatch(patch_mat, chrono.ChVectorD(-self.terrain_length/2, -self.terrain_width/2, 0), chrono.ChVectorD(
        #     0, 0, 1), self.terrain_length, self.terrain_width, self.terrain_thickness)
        patch = self.terrain.AddPatch(
            patch_mat, chrono.CSYSNORM, self.terrain_length, self.terrain_width)
        patch.SetTexture(self.chronopath +
                         '/sensor/textures/green_grass.jpg', 200, 200)
        patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
        self.terrain.Initialize()

        # ---------------------------
        # Provide a delta to controls
        # ---------------------------
        # Set the time response for steering and throttle inputs.
        steering_time = 0.75
        # time to go from 0 to +1 (or from 0 to -1)
        throttle_time = .5
        # time to go from 0 to +1
        braking_time = 0.3
        # time to go from 0 to +1
        self.SteeringDelta = (self._dyna_time_step / steering_time)
        self.ThrottleDelta = (self._dyna_time_step / throttle_time)
        self.BrakingDelta = (self._dyna_time_step / braking_time)

        # ---------------------------
        # Sensor stuff
        # ---------------------------
        self.manager = sens.ChSensorManager(self.system)
        self.manager.scene.AddPointLight(chrono.ChVectorF(
            0, 0, 100), chrono.ChColor(1, 1, 1), 5000)

        b = sens.Background()
        b.color_horizon = chrono.ChVectorF(.6, .7, .8)
        b.color_zenith = chrono.ChVectorF(.4, .5, .6)
        b.mode = sens.BackgroundMode_GRADIENT
        self.manager.scene.SetBackground(b)

        # GPS
        noise_model = sens.ChNoiseNone()
        gps_noise_model = sens.ChNoiseNormal(
            chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0.06, 0.06, 0.06))
        gps_offset_pose = chrono.ChFrameD(chrono.ChVectorD(
            0, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(1, 0, 0)))
        gps_reference = chrono.ChVectorD(-89.400, 43.070, 260.0)

        gps = sens.ChGPSSensor(self.vehicle.GetChassisBody(), 10,
                               gps_offset_pose, gps_reference, gps_noise_model)
        gps.SetName("GPS")
        gps.PushFilter(sens.ChFilterGPSAccess())
        self.manager.AddSensor(gps)

        # Magnetometer
        noise_model = sens.ChNoiseNone()
        imu_offset_pose = chrono.ChFrameD(chrono.ChVectorD(
            0, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(1, 0, 0)))
        mag = sens.ChMagnetometerSensor(self.vehicle.GetChassisBody(
        ), 100, imu_offset_pose, noise_model, gps_reference)
        mag.SetName("IMU - Magnetometer")
        mag.PushFilter(sens.ChFilterMagnetAccess())
        self.manager.AddSensor(mag)

        # Driver
        self.driver = veh.ChDriver(self.vehicle.GetVehicle())
        self.m_inputs = self.driver.GetInputs()

        # Get the observation
        # Get wpt after applying the look ahead distance
        yaw = chVector_to_npArray(self.initRot.Q_to_Euler123())[2]
        point_after_lookAhead = self.initLoc + \
            chrono.ChVectorD(self._lookAheadDist, self._lookAheadDist, self._lookAheadDist) * \
            chrono.ChVectorD(math.cos(yaw), math.sin(yaw), 0)

        # Get the closest points
        self.observation = self.get_obs(
            chVector_to_npArray(point_after_lookAhead)[:2])

        # This is the distance of the closest wpt from the vehicle -> Important since we need to know when to
        self._old_dist = np.linalg.norm(
            self.observation[0, :2] - chVector_to_npArray(self.chassis_body.GetPos())[:2], axis=0)
        speed = chVector_to_npArray(self.chassis_body.GetPos_dt())
        speed = np.linalg.norm(speed[0:2], axis=0)
        self._old_delta_speed = abs(self.observation[0, 3] - speed)

        self._terminated = False
        self._truncated = False
        self._isdone = (self._terminated or self._truncated)
        self.reward = 0

        return self.observation.flatten(), {}

    def step(self, action):
        """
        Perform a simulation step of the environment.
        :param action: The action to apply to the environment.
        :return: The observation, the reward, a boolean indicating if the episode is terminated, a boolean indicating if the episode is truncated, and a dictionary of info.
        """
        # Get the throttle and steering from the action
        throttle = action[0]
        if (throttle < 0):
            braking = abs(throttle)
            throttle = 0
        else:
            throttle = abs(throttle)
            braking = 0

        steering = action[1]

        # Update the driver inputs
        # self.driver.SetSteering(steering)
        # self.driver.SetThrottle(throttle)
        # self.driver.SetBraking(braking)

        # self.driver.SetSteering(0)
        # self.driver.SetThrottle(0)
        # self.driver.SetBraking(0)

        for i in range(round(1/(self._control_frequency*self._dyna_time_step))):
            time = self.system.GetChTime()
            self.m_inputs.m_steering = np.clip(steering, self.m_inputs.m_steering - self.SteeringDelta,
                                               self.m_inputs.m_steering + self.SteeringDelta)
            self.m_inputs.m_throttle = np.clip(throttle, self.m_inputs.m_throttle - self.ThrottleDelta,
                                               self.m_inputs.m_throttle + self.ThrottleDelta)
            self.m_inputs.m_braking = np.clip(braking,   self.m_inputs.m_braking - self.BrakingDelta,
                                              self.m_inputs.m_braking + self.BrakingDelta)

            # inputs.m_steering = steering
            # inputs.m_throttle = throttle
            # inputs.m_braking = braking
            # DEBUG
            # print('----------------------------------')
            # print('Throttle: ', self.m_inputs.m_throttle)
            # print('Steering: ', self.m_inputs.m_steering)
            # print('Braking: ', self.m_inputs.m_braking)

            # Sync the vehicle
            self.vehicle.Synchronize(
                time, self.m_inputs, self.terrain)
            self.terrain.Synchronize(time)

            # Advance the vehicle
            self.driver.Advance(self._dyna_time_step)
            self.vehicle.Advance(self._dyna_time_step)
            self.terrain.Advance(self._dyna_time_step)

            self.manager.Update()

        # Get wpt after applying the look ahead distance
        yaw = chVector_to_npArray(
            self.chassis_body.GetRot().Q_to_Euler123())[2]
        point_after_lookAhead = self.chassis_body.GetPos() + \
            chrono.ChVectorD(self._lookAheadDist, self._lookAheadDist, self._lookAheadDist) * \
            chrono.ChVectorD(math.cos(yaw), math.sin(yaw), 0)

        # Get the observation
        self.observation = self.get_obs(
            chVector_to_npArray(point_after_lookAhead)[:2])
        self.reward = self.get_reward()

        # Check if the episode is terminated
        self.is_truncated()
        self.is_terminated()
        self._isdone = (self._terminated or self._truncated)

        # print('Reward: ', self.reward)
        # print('Terminated: ', self._terminated)
        # print('Truncated: ', self._truncated)
        # print('----------------------------------')

        return self.observation.flatten(), self.reward, self._terminated, self._truncated, {}

    def render(self):
        width = 1280
        height = 720
        if not (self.play_mode == True):
            raise Exception('Please set play_mode=True to render')
        if not self.render_setup:

            self.vis_camera = sens.ChCameraSensor(
                self.chassis_body,  # body camera is attached to
                30,  # scanning rate in Hz
                chrono.ChFrameD(chrono.ChVectorD(-2, 0, .5), chrono.Q_from_AngAxis(
                    chrono.CH_C_PI / 20, chrono.ChVectorD(0, 1, 0))),
                # chrono.ChFrameD(chrono.ChVectorD(-2, 0, .5), chrono.Q_from_AngAxis(chrono.CH_C_PI, chrono.ChVectorD(0, 0, 1))),
                # offset pose
                width,  # number of horizontal samples
                height,  # number of vertical channels
                chrono.CH_C_PI / 3,  # horizontal field of view
            )
            self.vis_camera.SetName("Follow Camera Sensor")
            # self.camera.FilterList().append(sens.ChFilterVisualize(self.camera_width, self.camera_height, "RGB Camera"))
            # vis_camera.FilterList().append(sens.ChFilterVisualize(1280, 720, "Visualization Camera"))
            self.vis_camera.PushFilter(sens.ChFilterVisualize(
                width, height, "Visualization Camera"))
            self.vis_camera.PushFilter(sens.ChFilterRGBA8Access())
            self.manager.AddSensor(self.vis_camera)
            self.render_setup = True

        camera_data_RGBA8 = self.vis_camera.GetMostRecentRGBA8Buffer()
        if camera_data_RGBA8.HasData():
            rgb = camera_data_RGBA8.GetRGBA8Data()[:, :, 0:3]
        else:
            rgb = np.zeros((height, width, 3))

        return rgb
