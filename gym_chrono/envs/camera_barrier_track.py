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
from control_utilities.track import RandomTrack
from control_utilities.driver import Driver
from control_utilities.chrono_utilities import createChronoSystem

# openai-gym imports
import gym
from gym import spaces

def setDataDirectory():
    """
    Set data directory

    This is useful so data directory paths don't need to be changed everytime
    you pull from or push to github. To make this useful, make sure you perform
    step 2, as defined for your operating system.

    For Linux or Mac users:
      Replace bashrc with the shell your using. Could be .zshrc.
      1. echo 'export CHRONO_DATA_DIR=<chrono's data directory>' >> ~/.bashrc
          Ex. echo 'export CHRONO_DATA_DIR=/home/user/chrono/data/' >> ~/.zshrc
      2. source ~/.zshrc

    For Windows users:
      Link as reference: https://helpdeskgeek.com/how-to/create-custom-environment-variables-in-windows/
      1. Open the System Properties dialog, click on Advanced and then Environment Variables
      2. Under User variables, click New... and create a variable as described below
          Variable name: CHRONO_DATA_DIR
          Variable value: <chrono's data directory>
              Ex. Variable value: C:\ Users\ user\ chrono\ data\
    """
    from pathlib import Path

    CONDA_PREFIX = os.environ.get('CONDA_PREFIX')
    CHRONO_DATA_DIR = os.environ.get('CHRONO_DATA_DIR')
    if CONDA_PREFIX and not CHRONO_DATA_DIR:
        CHRONO_DATA_DIR = os.path.join(CONDA_PREFIX, 'share', 'chrono', 'data', '')
    if not CHRONO_DATA_DIR:
        CHRONO_DATA_DIR = os.path.join(Path(os.path.dirname(os.path.realpath(__file__))).parents[1], 'chrono', 'data', '')
    elif not CHRONO_DATA_DIR:
        raise Exception('Cannot find the chrono data directory. Please verify that CHRONO_DATA_DIR is set correctly.')

    chrono.SetChronoDataPath(CHRONO_DATA_DIR)
    veh.SetDataPath(os.path.join(CHRONO_DATA_DIR, 'vehicle', ''))

def GetInitPose(p1, p2, z=0.1, reversed=0):
    initLoc = p1

    vec = p2 - p1
    theta = math.atan2((vec%chrono.ChVectorD(1,0,0)).Length(),vec^chrono.ChVectorD(1,0,0))
    if reversed:
        theta *= -1
    initRot = chrono.ChQuaternionD()
    initRot.Q_from_AngZ(theta)

    return initLoc, initRot

class camera_barrier_track(ChronoBaseEnv):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}
    def __init__(self):
        ChronoBaseEnv.__init__(self)
        setDataDirectory()

        # Define action and observation space
        # They must be gym.spaces objects
        # Example when using discrete actions:
        self.camera_width  = 80*2
        self.camera_height = 45*2

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)
        self.observation_space = spaces.Box(low=0, high=255, shape=(self.camera_height, self.camera_width, 3), dtype=np.uint8)

        self.info =  {"timeout": 10000.0}
        self.timestep = 3e-3
        # ---------------------------------------------------------------------
        #
        #  Create the simulation system and add items
        #
        self.timeend = 30
        self.control_frequency = 10

        self.initLoc = chrono.ChVectorD(0, 0, .1)
        self.initRot = chrono.ChQuaternionD(1, 0, 0, 0)

        self.terrainHeight = 0  # terrain height (FLAT terrain only)
        self.terrainLength = 300.0  # size in X direction
        self.terrainWidth = 300.0  # size in Y direction

        self.target_speed = 5.0 # Target speed of the throttle controller

        # Track parameters
        self.track_x_max = 100
        self.track_y_max = 100
        self.track_width = 12.5

        self.render_setup = False
        self.play_mode = False

    def generate_track(self):
        reversed = randint(0,1)
        self.track = RandomTrack(x_max=self.track_x_max, y_max=self.track_y_max, width=self.track_width)
        self.track.generateTrack(seed=randint(0, 100), reversed=reversed)
        self.initLoc, self.initRot = GetInitPose(self.track.center.getPoint(0), self.track.center.getPoint(1), reversed=reversed)
        self.initLoc.z = 1

    def DrawBarriers(self, points, n=5, height=1, width=1):
        points = points[::n]
        if points[-1] != points[0]:
            points.append(points[-1])
        for i in range(len(points) - 1):
            p1 = points[i]
            p2 = points[i + 1]
            box = chrono.ChBodyEasyBox((p2 - p1).Length(), height, width, 1000, True, True)
            box.SetPos(p1)

            q = chrono.ChQuaternionD()
            v1 = p2 - p1
            v2 = chrono.ChVectorD(1, 0, 0)
            ang = math.atan2((v1 % v2).Length(), v1 ^ v2)
            if chrono.ChVectorD(0, 0, 1) ^ (v1 % v2) > 0.0:
                ang *= -1
            q.Q_from_AngZ(ang)
            box.SetRot(q)
            box.SetBodyFixed(True)

            box_asset = box.GetAssets()[0]
            visual_asset = chrono.CastToChVisualization(box_asset)

            vis_mat = chrono.ChVisualMaterial()
            vis_mat.SetAmbientColor(chrono.ChVectorF(0, 0, 0))

            if i % 2 == 0:
                vis_mat.SetDiffuseColor(chrono.ChVectorF(1.0, 0, 0))
            else:
                vis_mat.SetDiffuseColor(chrono.ChVectorF(1.0, 1.0, 1.0))
            vis_mat.SetSpecularColor(chrono.ChVectorF(0.9, 0.9, 0.9))
            vis_mat.SetFresnelMin(0)
            vis_mat.SetFresnelMax(0.1)

            visual_asset.material_list.append(vis_mat)

            color = chrono.ChColorAsset()
            if i % 2 == 0:
                color.SetColor(chrono.ChColor(1, 0, 0))
            else:
                color.SetColor(chrono.ChColor(1, 1, 1))
            box.AddAsset(color)
            self.system.Add(box)
            self.barriers.append(box)

    def reset(self):
        self.generate_track()
        self.vehicle = veh.Sedan()
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
        self.system = self.vehicle.GetSystem()
        self.chassis_body = self.vehicle.GetChassisBody()
        self.chassis_body.GetCollisionModel().ClearModel()
        size = chrono.ChVectorD(3,2,0.2)
        self.chassis_body.GetCollisionModel().AddBox(0.5 * size.x, 0.5 * size.y, 0.5 * size.z)
        self.chassis_body.GetCollisionModel().BuildModel()

        # Driver
        self.driver = Driver(self.vehicle.GetVehicle())

        # Throttle controller
        #self.throttle_controller = PIDThrottleController()
        #self.throttle_controller.SetGains(Kp=0.4, Ki=0, Kd=0)
        #self.throttle_controller.SetTargetSpeed(speed=self.target_speed)

        # Rigid terrain
        self.terrain = veh.RigidTerrain(self.system)
        patch = self.terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, self.terrainHeight - 5), chrono.QUNIT),
                                 chrono.ChVectorD(self.terrainLength, self.terrainWidth, 10))
        patch.SetContactFrictionCoefficient(0.9)
        patch.SetContactRestitutionCoefficient(0.01)
        patch.SetContactMaterialProperties(2e7, 0.3)
        patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
        patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
        self.terrain.Initialize()

        ground_body = patch.GetGroundBody()
        ground_asset = ground_body.GetAssets()[0]
        visual_asset = chrono.CastToChVisualization(ground_asset)
        vis_mat = chrono.ChVisualMaterial()
        vis_mat.SetKdTexture(chrono.GetChronoDataFile("concrete.jpg"))
        visual_asset.material_list.append(vis_mat)

        # create barriers
        self.barriers = []
        self.DrawBarriers(self.track.left.points)
        self.DrawBarriers(self.track.right.points)

        # Set the time response for steering and throttle inputs.
        # NOTE: this is not exact, since we do not render quite at the specified FPS.
        steering_time = 0.5
        # time to go from 0 to +1 (or from 0 to -1)
        throttle_time = 0.5
        # time to go from 0 to +1
        braking_time = 0.3
        # time to go from 0 to +1
        self.driver.SetSteeringDelta(self.timestep / steering_time)
        self.driver.SetThrottleDelta(self.timestep / throttle_time)
        self.driver.SetBrakingDelta(self.timestep / braking_time)

        self.manager = sens.ChSensorManager(self.system)
        self.manager.scene.AddPointLight(chrono.ChVectorF(100, 100, 100), chrono.ChVectorF(1, 1, 1), 500.0)
        self.manager.scene.AddPointLight(chrono.ChVectorF(-100, -100, 100), chrono.ChVectorF(1, 1, 1), 500.0)
        # ------------------------------------------------
        # Create a self.camera and add it to the sensor manager
        # ------------------------------------------------
        self.camera = sens.ChCameraSensor(
            self.chassis_body,  # body camera is attached to
            50,  # scanning rate in Hz
            chrono.ChFrameD(chrono.ChVectorD(-.075, 0, .15), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))),
            # offset pose
            self.camera_width,  # number of horizontal samples
            self.camera_height,  # number of vertical channels
            chrono.CH_C_PI / 2,  # horizontal field of view
            (self.camera_height / self.camera_width) * chrono.CH_C_PI / 3.  # vertical field of view
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

            self.driver.SetTargetSteering(self.ac[0,])
            self.driver.SetTargetThrottle(self.ac[1,])
            self.driver.SetTargetBraking(self.ac[2,])

            # Advance simulation for one timestep for all modules
            self.driver.Advance(self.timestep)
            self.vehicle.Advance(self.timestep)
            self.terrain.Advance(self.timestep)
            # self.system.DoStepDynamics(self.timestep)
            self.manager.Update()

            for barrier in self.barriers:
                self.c_f += barrier.GetContactForce().Length()

        self.rew = self.calc_rew()
        self.obs = self.get_ob()
        self.is_done()
        return self.obs, self.rew, self.isdone, self.info


    def get_ob(self):
        camera_data_RGBA8 = self.camera.GetMostRecentRGBA8Buffer()
        if camera_data_RGBA8.HasData():
            rgb = camera_data_RGBA8.GetRGBA8Data()[:,:,0:3]
            #rgb = cv2.flip(rgb, 0)
        else:
            rgb = np.zeros((self.camera_height,self.camera_width,3))
            #print('NO DATA \n')

        return rgb

    def calc_rew(self):
        dist_coeff = .1
        vel_coeff = 1
        err_coeff = -1
        time_cost = 0
        vel = self.vehicle.GetVehicle().GetVehicleSpeed()
        progress = self.calc_progress()
        pos = self.vehicle.GetVehicle().GetVehiclePos()
        err = (self.track.center.calcClosestPoint(pos) - pos).Length()
        rew = dist_coeff*progress + vel_coeff*vel + err_coeff*err + time_cost*self.system.GetChTime()
        return rew

    def calc_progress(self):
        dist = self.track.center.calcDistance(self.chassis_body.GetPos())
        progress = dist - self.old_dist
        self.old_dist = dist
        return progress

    def is_done(self):
        # print(self.c_f)
        collision = not(self.c_f == 0)
        if self.system.GetChTime() > self.timeend:
            self.isdone = True
        elif collision:
            self.rew += -200
            self.isdone = True


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
                # self.camera.FilterList().append(sens.ChFilterVisualize(self.camera_width, self.camera_height, "RGB Camera"))
                # vis_camera.FilterList().append(sens.ChFilterVisualize(1280, 720, "Visualization Camera"))
                if False:
                    self.camera.FilterList().append(sens.ChFilterSave())
                self.manager.AddSensor(vis_camera)

            if True:
                vis_camera = sens.ChCameraSensor(
                    self.chassis_body,  # body camera is attached to
                    30,  # scanning rate in Hz
                    chrono.ChFrameD(chrono.ChVectorD(-8, 0, 3), chrono.Q_from_AngAxis(chrono.CH_C_PI / 20, chrono.ChVectorD(0, 1, 0))),
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

    def close(self):
        del self

    def ScreenCapture(self, interval):
        raise NotImplementedError

    def __del__(self):
        del self.manager
