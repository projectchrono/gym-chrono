# PyChrono imports
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

# Default lib imports
import numpy as np
import math
import os
from random import randint

# Custom imports
from gym_chrono.envs.ChronoBase import ChronoBaseEnv
from gym_chrono.envs.utils.utilities import SetChronoDataDirectories, CalcInitialPose
from gym_chrono.envs.utils.pid_controller import PIDLongitudinalController

# openai-gym imports
import gym
from gym import spaces

# control_utilities imports
# from control_utilities.driver import Driver
# from control_utilities.track import RandomTrack

class rccar_cone_track(ChronoBaseEnv):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}
    def __init__(self):
        ChronoBaseEnv.__init__(self)

        # Set Chrono data directories
        SetChronoDataDirectories()

        # Define action and observation space
        # They must be gym.spaces objects
        # Example when using discrete actions:
        self.camera_width  = 80*2
        self.camera_height = 45*2
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)
        self.observation_space = spaces.Box(low=0, high=255, shape=(self.camera_height, self.camera_width, 3), dtype=np.uint8)

        self.info =  {"timeout": 10000.0}
        self.timestep = 1e-3

        # -------------------------------
        # Initialize simulation settings
        # -------------------------------

        self.timeend = 40
        self.control_frequency = 10

        self.terrainHeight = 0  # terrain height (FLAT terrain only)
        self.terrainLength = 300.0  # size in X direction
        self.terrainWidth = 300.0  # size in Y direction

        self.target_speed = 1.5 # target speed for PID throttle controller to maintain

        # Track parameters
        self.track_x_max = 100
        self.track_y_max = 100
        self.track_width = 1.1

        self.render_setup = False
        self.play_mode = False

    def generate_track(self, z=.15, starting_index=1):
        # Generate track
        reversed = randint(0,1)
        self.track = RandomTrack(x_max=self.track_x_max, y_max=self.track_y_max, width=self.track_width)
        self.track.generator.min_distance = 1
        self.track.generateTrack(reversed=reversed)

        # Calculate initial pose
        self.initLoc, self.initRot = CalcInitialPose(self.track.center.getPoint(starting_index), self.track.center.getPoint(starting_index+1))
        self.track.center.last_index = starting_index
        self.initLoc.z = z

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
        self.system = self.vehicle.GetSystem()

        # Driver
        self.driver = Driver(self.vehicle.GetVehicle())

        # Set the time response for steering and throttle inputs.
        # NOTE: this is not exact, since we do not render quite at the specified FPS.
        steering_time = .65
        # time to go from 0 to +1 (or from 0 to -1)
        throttle_time = .5
        # time to go from 0 to +1
        braking_time = 0.3
        # time to go from 0 to +1
        self.driver.SetSteeringDelta(self.timestep / steering_time)
        self.driver.SetThrottleDelta(self.timestep / throttle_time)
        self.driver.SetBrakingDelta(self.timestep / braking_time)

        # Throttle controller
        self.long_controller = PIDLongitudinalController(self.vehicle, self.driver)
        self.long_controller.SetGains(Kp=0.4, Ki=0, Kd=0)
        self.long_controller.SetTargetSpeed(speed=self.target_speed)

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
        self.manager = sens.ChSensorManager(self.system)
        self.manager.scene.AddPointLight(chrono.ChVectorF(100, 100, 100), chrono.ChVectorF(1, 1, 1), 500.0)
        self.manager.scene.AddPointLight(chrono.ChVectorF(-100, -100, 100), chrono.ChVectorF(1, 1, 1), 500.0)

        # create obstacles
        self.cones = []
        self.DrawCones(self.track.left.points, 'green')
        self.DrawCones(self.track.right.points, 'red')

        # -----------------------------------------------------
        # Create a self.camera and add it to the sensor manager
        # -----------------------------------------------------
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
        self.camera.FilterList().append(sens.ChFilterRGBA8Access())
        self.manager.AddSensor(self.camera)

        self.old_dist = self.track.center.calcDistance(self.chassis_body.GetPos())

        self.step_number = 0

        self.isdone = False
        self.render_setup = False
        if self.play_mode:
            self.render()

        return self.get_ob()

    def DrawCones(self, points, color, z=.31, n=10):
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
            self.manager.Update()

        self.rew = self.calc_rew()
        self.obs = self.get_ob()
        self.is_done()
        return self.obs, self.rew, self.isdone, self.info


    def get_ob(self):
        camera_data_RGBA8 = self.camera.GetMostRecentRGBA8Buffer()
        if camera_data_RGBA8.HasData():
            rgb = camera_data_RGBA8.GetRGBA8Data()[:,:,0:3]
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
        p.z = self.chassis_body.GetPos().z
        p = p-self.chassis_body.GetPos()

        if self.system.GetChTime() > self.timeend:
            self.isdone = True
        elif p.Length() > self.track.width / 2.75:
            self.rew += -200
            self.isdone = True

    def calc_progress(self):
        progress = self.track.center.calcDistance(self.chassis_body.GetPos()) - self.old_dist
        self.old_dist = self.track.center.calcDistance(self.chassis_body.GetPos())
        return progress

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
                    chrono.ChFrameD(chrono.ChVectorD(-2, 0, .5), chrono.Q_from_AngAxis(chrono.CH_C_PI / 20, chrono.ChVectorD(0, 1, 0))),
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
