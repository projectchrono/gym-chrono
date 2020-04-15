# PyChrono imports
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

# Default lib imports
import numpy as np
import math
import os
import random

# Custom imports
from gym_chrono.envs.ChronoBase import ChronoBaseEnv
from gym_chrono.envs.utils.perlin_bitmap_generator import generate_random_bitmap
from gym_chrono.envs.utils.utilities import SetChronoDataDirectories, CalcInitialPose

# openai-gym imports
import gym
from gym import spaces

class off_road(ChronoBaseEnv):
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

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)
        self.observation_space = spaces.Tuple((
                spaces.Box(low=0, high=255, shape=(self.camera_height, self.camera_width, 3), dtype=np.uint8),  # camera
                spaces.Box(low=-100000, high=300000, shape=(6,), dtype=np.float)))                                        # goal gps

        self.info =  {"timeout": 10000.0}
        self.timestep = 3e-3

        # -------------------------------
        # Initialize simulation settings
        # -------------------------------

        self.timeend = 40
        self.control_frequency = 10

        self.min_terrain_height = 0     # min terrain height
        self.max_terrain_height = 0 # max terrain height
        self.terrain_length = 100.0 # size in X direction
        self.terrain_width = 100.0  # size in Y direction

        self.initLoc = chrono.ChVectorD(-self.terrain_length / 2.25, -self.terrain_width / 2.25, self.max_terrain_height + 1)
        self.initRot = chrono.ChQuaternionD(1, 0, 0, 0)

        self.origin = chrono.ChVectorD(-89.400, 43.070, 260.0) # Origin being somewhere in Madison WI
        # used for converting to cartesian coordinates
        self.lat_rad = math.radians(self.origin.x)
        self.long_rad = math.radians(self.origin.y)
        self.lat_cos = math.cos(self.origin.x)


        self.render_setup = False
        self.play_mode = False

    def toCartesian(self, coord):
        """ Approximation: Converts GPS coordinate to x,y,z provided some origin """

        lat_rad = math.radians(coord.x)
        long_rad = math.radians(coord.y)
        EARTH_RADIUS = 6378.1e3 # [m]

        # x is East, y is North
        x = EARTH_RADIUS * (long_rad - self.long_rad) * self.lat_cos
        y = EARTH_RADIUS * (lat_rad - self.lat_rad)
        z = coord.z - self.origin.z

        return chrono.ChVectorD(x, y, z)

    def toGPSCoordinate(self, pos):
        """ Approximation: Converts x,y,z to GPS Coordinate provided some origin """

        EARTH_RADIUS = 6378.1e3 # [m]

        # x is East, y is North
        lat = math.degrees(pos.x / EARTH_RADIUS / self.lat_cos + self.long_rad)
        long = math.degrees(pos.y / EARTH_RADIUS + self.lat_rad)
        alt = pos.z + self.origin.z

        return chrono.ChVectorD(lat, long, alt)

    def reset(self):
        self.vehicle = veh.HMMWV_Reduced()
        self.vehicle.SetContactMethod(chrono.ChMaterialSurface.SMC)
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
        # self.chassis_body.GetCollisionModel().ClearModel()
        # size = chrono.ChVectorD(3,2,0.2)
        # self.chassis_body.GetCollisionModel().AddBox(0.5 * size.x, 0.5 * size.y, 0.5 * size.z)
        # self.chassis_body.GetCollisionModel().BuildModel()

        # Driver
        self.driver = veh.ChDriver(self.vehicle.GetVehicle())

        # Create the terrain
        self.bitmap_file =  os.getcwd() + "/height_map.bmp"
        self.bitmap_file_backup = os.getcwd() + "/height_map_backup.bmp"
        generate_random_bitmap(self.bitmap_file)

        self.terrain = veh.RigidTerrain(self.system)
        try:
            patch = self.terrain.AddPatch(chrono.CSYSNORM,       # position
                                        self.bitmap_file,        # heightmap file (.bmp)
                                        "test",                  # mesh name
                                        self.terrain_length,     # sizeX
                                        self.terrain_width,      # sizeY
                                        self.min_terrain_height, # hMin
                                        self.max_terrain_height) # hMax
        except Exception:
            print('Corrupt Bitmap File')
            patch = self.terrain.AddPatch(chrono.CSYSNORM,       # position
                                        self.bitmap_file_backup,        # heightmap file (.bmp)
                                        "test",                  # mesh name
                                        self.terrain_length,     # sizeX
                                        self.terrain_width,      # sizeY
                                        self.min_terrain_height, # hMin
                                        self.max_terrain_height) # hMax

        patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 16, 16)

        patch.SetContactFrictionCoefficient(0.9)
        patch.SetContactRestitutionCoefficient(0.01)
        patch.SetContactMaterialProperties(2e7, 0.3)
        patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
        self.terrain.Initialize()

        ground_body = patch.GetGroundBody()
        ground_asset = ground_body.GetAssets()[0]
        visual_asset = chrono.CastToChVisualization(ground_asset)
        visual_asset.SetStatic(True)
        vis_mat = chrono.ChVisualMaterial()
        vis_mat.SetKdTexture(veh.GetDataFile("terrain/textures/grass.jpg"))
        visual_asset.material_list.append(vis_mat)

        # create goal
        gx = random.random() * self.terrain_length - self.terrain_length / 2
        gy = random.random() * self.terrain_width - self.terrain_width / 2
        self.goal = chrono.ChVectorD(gx, gy, self.max_terrain_height + 1)
        self.goal_coord = self.toGPSCoordinate(self.goal)

        self.goal_sphere = chrono.ChBodyEasySphere(.25, 1000, False, True)
        self.goal_sphere.SetBodyFixed(True)
        self.goal_sphere.AddAsset(chrono.ChColorAsset(1,0,0))
        self.goal_sphere.SetPos(self.goal)
        if self.play_mode:
            self.system.Add(self.goal_sphere)

        # create obstacles


        # Set the time response for steering and throttle inputs.
        # NOTE: this is not exact, since we do not render quite at the specified FPS.
        steering_time = 0.75
        # time to go from 0 to +1 (or from 0 to -1)
        throttle_time = .5
        # time to go from 0 to +1
        braking_time = 0.3
        # time to go from 0 to +1
        self.SteeringDelta = (self.timestep / steering_time)
        self.ThrottleDelta = (self.timestep / throttle_time)
        self.BrakingDelta = (self.timestep / braking_time)

        self.manager = sens.ChSensorManager(self.system)
        self.manager.scene.AddPointLight(chrono.ChVectorF(100, 100, 100), chrono.ChVectorF(1, 1, 1), 500.0)
        self.manager.scene.AddPointLight(chrono.ChVectorF(-100, -100, 100), chrono.ChVectorF(1, 1, 1), 500.0)

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
        self.manager.AddSensor(self.camera)
        self.camera.FilterList().append(sens.ChFilterRGBA8Access())

        # -----------------------------------------------------
        # Create a self.gps and add it to the sensor manager
        # -----------------------------------------------------
        gps_noise_none = sens.ChGPSNoiseNone()
        self.gps = sens.ChGPSSensor(
            self.chassis_body,
            100,
            chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))),
            self.origin,
            gps_noise_none
        )
        self.gps.SetName("GPS Sensor")
        self.gps.FilterList().append(sens.ChFilterGPSAccess())
        self.manager.AddSensor(self.gps)

        self.old_dist = (self.goal - self.initLoc).Length()
        self.cur_coord = self.origin

        self.step_number = 0
        self.c_f = 0
        self.isdone = False
        self.render_setup = False
        if self.play_mode:
            self.render()

        # print(self.get_ob()[1])
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

            steering = np.clip(self.ac[0,], self.driver.GetSteering() - self.SteeringDelta, self.driver.GetSteering() + self.SteeringDelta)
            if self.ac[1,] > 0:
                throttle = np.clip(abs(self.ac[1,]), self.driver.GetThrottle() - self.ThrottleDelta, self.driver.GetThrottle() + self.ThrottleDelta)
                braking = np.clip(0, self.driver.GetBraking() - self.BrakingDelta, self.driver.GetBraking() + self.BrakingDelta)
            else:
                braking = np.clip(abs(self.ac[2,]), self.driver.GetBraking() - self.BrakingDelta, self.driver.GetBraking() + self.BrakingDelta)
                throttle = np.clip(0, self.driver.GetThrottle() - self.ThrottleDelta, self.driver.GetThrottle() + self.ThrottleDelta)

            self.driver.SetSteering(steering)
            self.driver.SetThrottle(throttle)
            self.driver.SetBraking(braking)

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
        camera_buffer_RGBA8 = self.camera.GetMostRecentRGBA8Buffer()
        if camera_buffer_RGBA8.HasData():
            rgb = camera_buffer_RGBA8.GetRGBA8Data()[:,:,0:3]
            rgb = np.zeros((self.camera_height,self.camera_width,3))
        else:
            rgb = np.zeros((self.camera_height,self.camera_width,3))
            #print('NO DATA \n')

        gps_buffer = self.gps.GetMostRecentGPSBuffer()
        if gps_buffer.HasData():
            cur_gps_data = gps_buffer.GetGPSData()[0:3]
            self.cur_coord = chrono.ChVectorD(cur_gps_data[0], cur_gps_data[1], cur_gps_data[2])
        else:
            cur_gps_data = np.array([self.origin.x, self.origin.y, self.origin.z])

        goal_gps_data = np.array([self.goal_coord.x, self.goal_coord.y, self.goal_coord.z])

        gps_data = np.concatenate([cur_gps_data, goal_gps_data]) * 1000

        return (rgb, gps_data)

    def calc_rew(self):
        progress_coeff = 5
        vel_coeff = 1
        time_cost = 0
        progress = self.calc_progress()
        vel = self.vehicle.GetVehicle().GetVehicleSpeed()
        rew = progress_coeff*progress + vel_coeff*vel + time_cost*self.system.GetChTime()
        return rew

    def is_done(self):

        pos = self.chassis_body.GetPos()

        collision = not(self.c_f == 0)
        if self.system.GetChTime() > self.timeend:
            self.isdone = True
        elif abs(pos.x) > self.terrain_length / 2.0 or abs(pos.y) > self.terrain_width / 2 or pos.z < self.min_terrain_height:
            self.rew -= 200
            self.isdone = True
        elif (self.chassis_body.GetPos() - self.goal).Length() < 5:
            self.rew += 2500
            self.isdone = True

    def calc_progress(self):
        dist = (self.chassis_body.GetPos() - self.goal).Length()
        progress = self.old_dist - dist
        self.old_dist = dist
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
