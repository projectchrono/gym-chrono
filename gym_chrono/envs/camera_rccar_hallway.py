# PyChrono imports
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

# Default lib imports
import numpy as np
import math
import os
from random import randint
#import cv2

# Custom imports
from gym_chrono.envs.ChronoBase import ChronoBaseEnv
from control_utilities.chrono_utilities import calcPose, setDataDirectory
from control_utilities.driver import Driver
from control_utilities.track import Track
from gym_chrono.envs.utils.pid_controller import PIDLongitudinalController

# openai-gym imports
import gym
from gym import spaces

import cv2

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
class camera_rccar_hallway(ChronoBaseEnv):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}
    def __init__(self):
        ChronoBaseEnv.__init__(self)
        setDataDirectory()

        # Define action and observation space
        # They must be gym.spaces objects
        # Example when using discrete actions:
        self.camera_width  = 210
        self.camera_height = 160
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)
        self.observation_space = spaces.Box(low=0, high=255, shape=(self.camera_height, self.camera_width, 3), dtype=np.uint8)

        self.info =  {"timeout": 10000.0}
        self.timestep = 5e-3
        # ---------------------------------------------------------------------
        #
        #  Create the simulation system and add items
        #
        self.timeend = 100
        self.control_frequency = 10

        self.initLoc = chrono.ChVectorD(0, 0, .1)
        self.initRot = chrono.ChQuaternionD(1, 0, 0, 0)

        self.target_speed = .75

        self.render_setup = False
        self.play_mode = False
        self.step_number = 0

    def generate_track(self, starting_index=1):
        points = [[-8.713, -1.646],
                  [-7.851, -1.589],
                  [-6.847, -1.405],
                  [-6.048, -1.449],
                  [-5.350, -1.658],
                  [-4.628, -1.767],
                  [-3.807, -1.789],
                  [-2.865, -1.778],
                  [-1.823, -1.743],
                  [-0.724, -1.691],
                  [0.373, -1.650],
                  [1.411, -1.527],
                  [2.349, -1.453],
                  [3.174, -1.439],
                  [3.915, -1.474],
                  [4.652, -1.513],
                  [5.487, -1.694],
                  [6.506, -1.756],
                  [7.506, -1.456],
                  [7.732, -1.060],
                  [7.983, -0.617],
                  [7.432, 1.112],
                  [6.610, 1.143],
                  [5.688, 1.206],
                  [4.950, 1.281],
                  [4.331, 1.337],
                  [3.754, 1.349],
                  [3.152, 1.303],
                  [2.478, 1.207],
                  [1.708, 1.077],
                  [0.832, 0.940],
                  [-0.143, 0.828],
                  [-1.201, 0.767],
                  [-2.318, 0.781],
                  [-3.463, 0.830],
                  [-4.605, 0.838],
                  [-5.715, 0.864],
                  [-6.765, 0.934],
                  [-7.737, 1.121],
                  [-8.822, 1.318],
                  [-10.024, 0.608],
                  [-10.102, 0.437],
                  [-10.211, -0.569],
                  [-9.522, -1.514],
                  [-8.713, -1.646]]
        width = 1.1
        self.track = Track(points, width=width)
        self.track.generateTrack()
        self.initLoc, self.initRot = calcPose(self.track.center.getPoint(starting_index), self.track.center.getPoint(starting_index+1))
        self.track.center.last_index = starting_index
        self.initLoc.z = 0.5

    def reset(self):
        self.generate_track(starting_index=350)
        self.vehicle = veh.RCCar()
        self.vehicle.SetContactMethod(chrono.ChMaterialSurface.NSC)
        self.vehicle.SetChassisCollisionType(veh.ChassisCollisionType_NONE)

        self.vehicle.SetChassisFixed(False)
        self.vehicle.SetInitPosition(chrono.ChCoordsysD(self.initLoc, self.initRot))
        self.vehicle.SetTireType(veh.TireModelType_RIGID)
        self.vehicle.SetTireStepSize(self.timestep)
        self.vehicle.Initialize()

        self.vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
        self.vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
        self.vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
        self.vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
        self.vehicle.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)
        self.chassis_body = self.vehicle.GetChassisBody()
        self.system = self.vehicle.GetVehicle().GetSystem()

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

        # Longitudinal controller (throttle and braking)
        self.long_controller = PIDLongitudinalController(self.vehicle, self.driver)
        self.long_controller.SetGains(Kp=0.4, Ki=0, Kd=0)
        self.long_controller.SetTargetSpeed(speed=0.75)

        # Mesh hallway 
        y_max = 5.65
        x_max = 23
        offset = chrono.ChVectorD(-x_max/2, -y_max/2, .21)
        offsetF = chrono.ChVectorF(offset.x, offset.y, offset.z)

        self.terrain = veh.RigidTerrain(self.system)
        coord_sys = chrono.ChCoordsysD(offset, chrono.ChQuaternionD(1,0,0,0))
        patch = self.terrain.AddPatch(coord_sys, chrono.GetChronoDataFile("sensor/textures/hallway.obj"), "mesh", 0.01, False)


        vis_mesh = chrono.ChTriangleMeshConnected()
        vis_mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("sensor/textures/hallway.obj"), True, True)

        trimesh_shape = chrono.ChTriangleMeshShape()
        trimesh_shape.SetMesh(vis_mesh)
        trimesh_shape.SetName("mesh_name")
        trimesh_shape.SetStatic(True)

        patch.GetGroundBody().AddAsset(trimesh_shape)

        patch.SetContactFrictionCoefficient(0.9)
        patch.SetContactRestitutionCoefficient(0.01)
        patch.SetContactMaterialProperties(2e7, 0.3)

        self.terrain.Initialize()

        # create obstacles
        self.DrawCones(self.track.left.points, 'green')
        self.DrawCones(self.track.right.points, 'red')

        f = 0
        start_light = 4
        end_light = 5
        self.manager = sens.ChSensorManager(self.system)
        for i in range(8):
            f += 3
            if i <= start_light or i > end_light:
                continue
            self.manager.scene.AddPointLight(chrono.ChVectorF(f,1.25,2.3)+offsetF,chrono.ChVectorF(1,1,1),10)
            self.manager.scene.AddPointLight(chrono.ChVectorF(f,3.75,2.3)+offsetF,chrono.ChVectorF(1,1,1),10)
        # ------------------------------------------------
        # Create a self.camera and add it to the sensor manager
        # ------------------------------------------------
        self.camera = sens.ChCameraSensor(
            self.chassis_body,  # body camera is attached to
            30,  # scanning rate in Hz
            chrono.ChFrameD(chrono.ChVectorD(-.075, 0, .15), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))),
            # offset pose
            self.camera_width,  # number of horizontal samples
            self.camera_height,  # number of vertical channels
            chrono.CH_C_PI / 4,  # horizontal field of view
            (self.camera_height / self.camera_width) * chrono.CH_C_PI / 4.  # vertical field of view
        )
        self.camera.SetName("Camera Sensor")
        self.manager.AddSensor(self.camera)

        # -----------------------------------------------------------------
        # Create a filter graph for post-processing the data from the lidar
        # -----------------------------------------------------------------

        self.camera.FilterList().append(sens.ChFilterRGBA8Access())

        self.old_dist = self.track.center.calcDistance(self.chassis_body.GetPos())

        self.step_number = 0
        self.c_f = 0
        self.isdone = False
        self.render_setup = False
        if self.play_mode:
            self.render()

        return self.get_ob()

    def DrawCones(self, points, color, z=.3, n=10):
        for p in points[::n]:
            p.z += z
            cmesh = chrono.ChTriangleMeshConnected()
            if color=='red':
                cmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("sensor/cones/red_cone.obj"), False, True)
            elif color=='green':
                cmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("sensor/cones/green_cone.obj"), False, True)

            cshape = chrono.ChTriangleMeshShape()
            cshape.SetMesh(cmesh)
            cshape.SetName("Cone")
            cshape.SetStatic(True)

            cbody = chrono.ChBody()
            cbody.SetPos(p)
            cbody.AddAsset(cshape)
            cbody.SetBodyFixed(True)
            cbody.SetCollide(False)
            if color=='red':
                cbody.AddAsset(chrono.ChColorAsset(1,0,0))
            elif color=='green':
                cbody.AddAsset(chrono.ChColorAsset(0,1,0))

            self.system.Add(cbody)

    def step(self, ac):
        self.ac = ac.reshape((-1,))
        # Collect output data from modules (for inter-module communication)

        for i in range(round(1/(self.control_frequency*self.timestep))):
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

        self.rew = self.calc_rew()
        self.obs = self.get_ob()
        self.is_done()
        return self.obs, self.rew, self.isdone, self.info


    def get_ob(self):
        camera_data_RGBA8 = self.camera.GetMostRecentRGBA8Buffer()
        if camera_data_RGBA8.HasData():
            rgb = camera_data_RGBA8.GetRGBA8Data()[:,:,0:3]
            # print('Image Saving {}'.format(self.step_number))
            # cv2.imwrite('frame{}.png'.format(self.step_number), rgb)
            # self.step_number += 1
            #rgb = cv2.flip(rgb, 0)
        else:
            rgb = np.zeros((self.camera_height,self.camera_width,3))
            #print('NO DATA \n')

        return rgb

    def calc_rew(self):
        dist_coeff = 10
        time_coeff = 0
        progress = self.calc_progress()
        rew = dist_coeff*progress + time_coeff*self.system.GetChTime()
        return rew

    def is_done(self):

        p = self.track.center.calcClosestPoint(self.chassis_body.GetPos())
        p = p-self.chassis_body.GetPos()

        collision = not(self.c_f == 0)
        if self.system.GetChTime() > self.timeend:
            print("Over self.timeend")
            self.isdone = True
        elif p.Length() > self.track.width / 2.25:
            # self.rew += -200
            self.isdone = True
        # elif self.chassis_body.GetPos().x > self.Xtarg :
        #     self.rew += 1000
        #     self.isdone = True


    def render(self, mode='human'):
        if not (self.play_mode==True):
            raise Exception('Please set play_mode=True to render')

        if not self.render_setup:
            if False:
                vis_camera = sens.ChCameraSensor(
                    self.chassis_body,  # body camera is attached to
                    30,  # scanning rate in Hz
                    chrono.ChFrameD(chrono.ChVectorD(0, 0, 25), chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 1, 0))),
                    # offset pose
                    1280,  # number of horizontal samples
                    720,  # number of vertical channels
                    chrono.CH_C_PI / 3,  # horizontal field of view
                    (720/1280) * chrono.CH_C_PI / 3.  # vertical field of view
                )
                vis_camera.SetName("Birds Eye Camera Sensor")
                self.camera.FilterList().append(sens.ChFilterVisualize(self.camera_width, self.camera_height, "RGB Camera"))
                vis_camera.FilterList().append(sens.ChFilterVisualize(1280, 720, "Visualization Camera"))
                if False:
                    self.camera.FilterList().append(sens.ChFilterSave())
                self.manager.AddSensor(vis_camera)

            if True:
                vis_camera = sens.ChCameraSensor(
                    self.chassis_body,  # body camera is attached to
                    30,  # scanning rate in Hz
                    chrono.ChFrameD(chrono.ChVectorD(-2, 0, .5), chrono.Q_from_AngAxis(chrono.CH_C_PI / 20, chrono.ChVectorD(0, 1, 0))),
                    # chrono.ChFrameD(chrono.ChVectorD(-2, 0, .5), chrono.Q_from_AngAxis(chrono.CH_C_PI, chrono.ChVectorD(0, 0, 1))),
                    # offset pose
                    1280,  # number of horizontal samples
                    720,  # number of vertical channels
                    chrono.CH_C_PI / 3,  # horizontal field of view
                    (720/1280) * chrono.CH_C_PI / 3.  # vertical field of view
                )
                vis_camera.SetName("Follow Camera Sensor")
                # self.camera.FilterList().append(sens.ChFilterVisualize(self.camera_width, self.camera_height, "RGB Camera"))
                # vis_camera.FilterList().append(sens.ChFilterVisualize(1280, 720, "Visualization Camera"))
                if True:
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
        progress = self.track.center.calcDistance(self.chassis_body.GetPos())# - self.old_dist
        self.old_dist = self.track.center.calcDistance(self.chassis_body.GetPos())
        return progress

    def close(self):
        del self

    def ScreenCapture(self, interval):
        raise NotImplementedError

    def __del__(self):
        del self.manager
