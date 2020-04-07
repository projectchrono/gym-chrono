# PyChrono imports
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

# Default lib imports
import numpy as np
import math
import os
from random import randint
# import cv2

# Custom imports
from gym_chrono.envs.ChronoBase import ChronoBaseEnv
from control_utilities.chrono_utilities import calcPose, setDataDirectory
from control_utilities.driver import Driver
from control_utilities.track import Track
from gym_chrono.envs.utils.pid_controller import PIDLongitudinalController

# openai-gym imports
import gym
from gym import spaces

import math as m


# ----------------------------------------------------------------------------------------------------
# Set data directory
#
# This is useful so data directory paths don't need to be changed everytime
# you pull from or push to github. To make this useful, make sure you perform
# step 2, as defined for your operating system.
#
# For Linux or Mac users:
#   Replace bashrc with the shell your using. Could be .zshrc.
#   1. echo 'export CHRONO_DATA_DIR=<chrono's data directory>' >> ~/.bashrc
#       Ex. echo 'export CHRONO_DATA_DIR=/home/user/chrono/data/' >> ~/.zshrc
#   2. source ~/.zshrc
#
# For Windows users:
#   Link as reference: https://helpdeskgeek.com/how-to/create-custom-environment-variables-in-windows/
#   1. Open the System Properties dialog, click on Advanced and then Environment Variables
#   2. Under User variables, click New... and create a variable as described below
#       Variable name: CHRONO_DATA_DIR
#       Variable value: <chrono's data directory>
#           Ex. Variable value: C:\Users\user\chrono\data\
# ----------------------------------------------------------------------------------------------------

class BezierPath(chrono.ChBezierCurve):
    def __init__(self, beginPos, endPos, z):
        # making 4 turns to get to the end point
        deltaX = (endPos[0] - beginPos[0])/3
        deltaY = (endPos[1] - beginPos[1])/2
        points = chrono.vector_ChVectorD()
        for i in range(6):
            point = chrono.ChVectorD(beginPos[0] + deltaX*m.floor((i+1)/2) , beginPos[1] + deltaY*m.floor(i/2), z)
            points.append(point)

        self.current_t = 0
        super(BezierPath, self).__init__(points)

    # Update the progress on the path of the leader
    def Advance(self, delta_t):
        self.current_t += delta_t

    # Param-only derivative
    def par_evalD(self, t):
        par = np.clip(t, 0.0, 1.0)
        numIntervals = self.getNumPoints() - 1
        epar = par * numIntervals
        i = m.floor(par * numIntervals)
        i = np.clip(i, 0, numIntervals - 1)
        return self.evalD(int(i), epar - i)

    # Current positon and rotation of the leader vehicle chassis
    def getState(self):
        pos = self.eval(self.current_t)
        posD = self.par_evalD(self.current_t)
        alpha = m.atan2(posD.y, posD.x)
        rot = chrono.Q_from_AngZ(alpha)
        return pos, rot

class GVSETS_env(ChronoBaseEnv):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self):
        ChronoBaseEnv.__init__(self)
        setDataDirectory()

        # Define action and observation space
        # They must be gym.spaces objects
        # Example when using discrete actions:
        self.camera_width = 210
        self.camera_height = 160
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)
        self.observation_space = spaces.Box(low=0, high=255, shape=(self.camera_height, self.camera_width, 3),
                                            dtype=np.uint8)

        self.info = {"timeout": 10000.0}
        self.timestep = 5e-3
        # ---------------------------------------------------------------------
        #
        #  Create the simulation system and add items
        #
        self.timeend = 100
        self.control_frequency = 10

        self.initLoc = chrono.ChVectorD(0, 0, .1)
        self.initRot = chrono.ChQuaternionD(1, 0, 0, 0)
        leader_initloc = [-90, -40]
        leader_endloc = [90, 40]
        # time needed by the leader to get to the end of the path
        self.leader_totaltime = 20/5e-3
        self.path = BezierPath(leader_initloc, leader_endloc, 10.0)
        self.terrain_model = veh.RigidTerrain.PatchType_BOX
        self.terrainHeight = 0  # terrain height (FLAT terrain only)
        self.terrainLength = 150.0  # size in X direction
        self.terrainWidth = 150.0  # size in Y direction


        self.render_setup = False
        self.play_mode = False
        self.step_number = 0


    def reset(self):
        self.vehicle = veh.HMMWV_Reduced()
        self.vehicle.SetContactMethod(chrono.ChMaterialSurface.NSC)
        self.vehicle.SetChassisCollisionType(veh.ChassisCollisionType_PRIMITIVES)

        self.vehicle.SetChassisFixed(False)
        self.vehicle.SetInitPosition(chrono.ChCoordsysD(self.initLoc, self.initRot))
        self.vehicle.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
        self.vehicle.SetDriveType(veh.DrivelineType_AWD)
        # self.vehicle.SetSteeringType(veh.SteeringType_PITMAN_ARM)
        self.vehicle.SetTireType(veh.TireModelType_TMEASY)
        self.vehicle.SetTireStepSize(self.timestep)
        self.vehicle.Initialize()

        # self.vehicle.SetStepsize(self.timestep)
        if self.play_mode == True:
            self.vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
            self.vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
        else:
            self.vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
            self.vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
        self.vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
        self.vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
        self.chassis_body = self.vehicle.GetChassisBody()
        self.chassis_body.GetCollisionModel().ClearModel()
        size = chrono.ChVectorD(3, 2, 0.2)
        self.chassis_body.GetCollisionModel().AddBox(0.5 * size.x, 0.5 * size.y, 0.5 * size.z)
        self.chassis_body.GetCollisionModel().BuildModel()
        self.system = self.vehicle.GetVehicle().GetSystem()
        self.manager = sens.ChSensorManager(self.system)
        self.manager.scene.AddPointLight(chrono.ChVectorF(100, 100, 100), chrono.ChVectorF(1, 1, 1), 500.0)
        self.manager.scene.AddPointLight(chrono.ChVectorF(-100, -100, 100), chrono.ChVectorF(1, 1, 1), 500.0)
        # Driver
        self.driver = Driver(self.vehicle.GetVehicle())

        # Set the time response for steering and throttle inputs.
        # NOTE: this is not exact, since we do not render quite at the specified FPS.
        steering_time = 1.0
        # time to go from 0 to +1 (or from 0 to -1)
        throttle_time = .5
        # time to go from 0 to +1
        braking_time = 0.3
        # time to go from 0 to +1
        self.driver.SetSteeringDelta(self.timestep / steering_time)
        self.driver.SetThrottleDelta(self.timestep / throttle_time)
        self.driver.SetBrakingDelta(self.timestep / braking_time)
        #self.driver.SetGains(4, 4, 4)

        # Longitudinal controller (throttle and braking)
        self.long_controller = PIDLongitudinalController(self.vehicle, self.driver)
        self.long_controller.SetGains(Kp=0.4, Ki=0, Kd=0)
        self.long_controller.SetTargetSpeed(speed=0.75)

        # Mesh hallway
        y_max = 5.65
        x_max = 23
        offset = chrono.ChVectorD(-x_max / 2, -y_max / 2, .21)
        offsetF = chrono.ChVectorF(offset.x, offset.y, offset.z)

        self.terrain = veh.RigidTerrain(self.system)
        patch = self.terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, self.terrainHeight - 5), chrono.QUNIT),
                                      chrono.ChVectorD(self.terrainLength, self.terrainWidth, 10))
        patch.SetContactFrictionCoefficient(0.9)
        patch.SetContactRestitutionCoefficient(0.01)
        patch.SetContactMaterialProperties(2e7, 0.3)
        patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
        patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
        self.terrain.Initialize()
        self.groundBody = patch.GetGroundBody()
        # Leader
        self.leader = chrono.ChBody()
        vis_mesh = chrono.ChTriangleMeshConnected()
        vis_mesh.LoadWavefrontMesh(veh.GetDataFile("hmmwv/hmmwv_chassis.obj"), True, True)

        trimesh_shape = chrono.ChTriangleMeshShape()
        trimesh_shape.SetMesh(vis_mesh)
        trimesh_shape.SetName("mesh_name")
        trimesh_shape.SetStatic(True)

        self.leader.AddAsset(trimesh_shape)
        self.system.Add(self.leader)

        # ------------------------------------------------
        # Create a self.camera and add it to the sensor manager
        # ------------------------------------------------
        self.camera = sens.ChCameraSensor(
            self.chassis_body,  # body camera is attached to
            30,  # scanning rate in Hz
            chrono.ChFrameD(chrono.ChVectorD(1.5, 0, .875)),
            # offset pose
            self.camera_width,  # number of horizontal samples
            self.camera_height,  # number of vertical channels
            chrono.CH_C_PI / 3,  # horizontal field of view
            (self.camera_height / self.camera_width) * chrono.CH_C_PI / 3.  # vertical field of view
        )
        self.camera.SetName("Camera Sensor")
        self.manager.AddSensor(self.camera)

        # -----------------------------------------------------------------
        # Create a filter graph for post-processing the data from the lidar
        # -----------------------------------------------------------------

        self.camera.FilterList().append(sens.ChFilterRGBA8Access())

        #self.old_dist = self.track.center.calcDistance(self.chassis_body.GetPos())

        self.step_number = 0
        self.c_f = 0
        self.isdone = False
        self.render_setup = False
        if self.play_mode:
            self.render()

        return self.get_ob()

    def step(self, ac):

        self.ac = ac.reshape((-1,))
        # Collect output data from modules (for inter-module communication)

        for i in range(round(1 / (self.control_frequency * self.timestep))):
            self.driver_inputs = self.driver.GetInputs()
            # Update modules (process inputs from other modules)
            time = self.system.GetChTime()
            self.driver.Synchronize(time)
            self.vehicle.Synchronize(time, self.driver_inputs, self.terrain)
            self.terrain.Synchronize(time)

            throttle, braking = self.long_controller.Advance(self.timestep)
            self.driver.SetTargetThrottle(throttle)
            self.driver.SetTargetBraking(braking)
            self.driver.SetTargetSteering(self.ac[0,])

            # Advance simulation for one timestep for all modules
            self.driver.Advance(self.timestep)
            self.vehicle.Advance(self.timestep)
            self.terrain.Advance(self.timestep)
            self.system.DoStepDynamics(self.timestep)
            self.manager.Update()

            leaderPos, leaderRot = self.path.getState()
            self.leader.SetPos(leaderPos)
            # TODO set proper rot
            self.leader.SetRot(leaderRot)
            self.path.Advance(1 / self.leader_totaltime)

        self.rew = self.calc_rew()
        self.obs = self.get_ob()
        self.is_done()
        return self.obs, self.rew, self.isdone, self.info

    def get_ob(self):
        camera_data_RGBA8 = self.camera.GetMostRecentRGBA8Buffer()
        if camera_data_RGBA8.HasData():
            rgb = camera_data_RGBA8.GetRGBA8Data()[:, :, 0:3]
            # print('Image Saving {}'.format(self.step_number))
            # cv2.imwrite('frame{}.png'.format(self.step_number), rgb)
            # self.step_number += 1
            # rgb = cv2.flip(rgb, 0)
        else:
            rgb = np.zeros((self.camera_height, self.camera_width, 3))
            # print('NO DATA \n')

        return rgb

    def calc_rew(self):
        dist_coeff = 10
        time_coeff = 0
        progress = 0#self.calc_progress()
        rew = dist_coeff * progress + time_coeff * self.system.GetChTime()
        return rew

    def is_done(self):
        pass
        """
        p = self.track.center.calcClosestPoint(self.chassis_body.GetPos())
        p = p - self.chassis_body.GetPos()

        collision = not (self.c_f == 0)
        if self.system.GetChTime() > self.timeend:
            print("Over self.timeend")
            self.isdone = True
        elif p.Length() > self.track.width / 2.25:
            self.isdone = True
        """
    def render(self, mode='human'):
        if not (self.play_mode == True):
            raise Exception('Please set play_mode=True to render')

        if not self.render_setup:
            if True:
                vis_camera = sens.ChCameraSensor(
                    self.groundBody,  # body camera is attached to
                    30,  # scanning rate in Hz
                    chrono.ChFrameD(chrono.ChVectorD(0, 0, 225), chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 1, 0))),
                    # offset pose
                    1280,  # number of horizontal samples
                    720,  # number of vertical channels
                    chrono.CH_C_PI / 3,  # horizontal field of view
                    (720 / 1280) * chrono.CH_C_PI / 3.  # vertical field of view
                )
                vis_camera.SetName("Birds Eye Camera Sensor")
                self.camera.FilterList().append(
                    sens.ChFilterVisualize(self.camera_width, self.camera_height, "RGB Camera"))
                vis_camera.FilterList().append(sens.ChFilterVisualize(1280, 720, "Visualization Camera"))
                if False:
                    self.camera.FilterList().append(sens.ChFilterSave())
                self.manager.AddSensor(vis_camera)

            if False:
                vis_camera = sens.ChCameraSensor(
                    self.chassis_body,  # body camera is attached to
                    30,  # scanning rate in Hz
                    chrono.ChFrameD(chrono.ChVectorD(-2, 0, .5),
                                    chrono.Q_from_AngAxis(chrono.CH_C_PI / 20, chrono.ChVectorD(0, 1, 0))),
                    # chrono.ChFrameD(chrono.ChVectorD(-2, 0, .5), chrono.Q_from_AngAxis(chrono.CH_C_PI, chrono.ChVectorD(0, 0, 1))),
                    # offset pose
                    1280,  # number of horizontal samples
                    720,  # number of vertical channels
                    chrono.CH_C_PI / 3,  # horizontal field of view
                    (720 / 1280) * chrono.CH_C_PI / 3.  # vertical field of view
                )
                vis_camera.SetName("Follow Camera Sensor")
                self.camera.FilterList().append(
                    sens.ChFilterVisualize(self.camera_width, self.camera_height, "RGB Camera"))
                vis_camera.FilterList().append(sens.ChFilterVisualize(1280, 720, "Visualization Camera"))
                if False:
                    vis_camera.FilterList().append(sens.ChFilterSave())
                self.manager.AddSensor(vis_camera)

            # -----------------------------------------------------------------
            # Create a filter graph for post-processing the data from the lidar
            # -----------------------------------------------------------------

            # self.camera.FilterList().append(sens.ChFilterVisualize("RGB Camera"))
            # vis_camera.FilterList().append(sens.ChFilterVisualize("Visualization Camera"))
            self.render_setup = True

        if (mode == 'rgb_array'):
            return self.get_ob()

    def calc_progress(self):
        progress = self.track.center.calcDistance(self.chassis_body.GetPos())  # - self.old_dist
        self.old_dist = self.track.center.calcDistance(self.chassis_body.GetPos())
        return progress

    def close(self):
        del self

    def ScreenCapture(self, interval):
        raise NotImplementedError

    def __del__(self):
        del self.manager
