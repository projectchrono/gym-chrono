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
from gym_chrono.envs.utils.utilities import SetChronoDataDirectories, CalcInitialPose, areColliding
from gym_chrono.envs.utils.gps_utilities import *

# openai-gym imports
import gym
from gym import spaces
import time as t

class AssetMesh():
    def __init__(self, filename, bounding_box=None):
        self.filename = filename

        # If bounding box is not passed in, calculate it
        if bounding_box == None:
            self.bounding_box = CalcBoundingBox()
        else:
            self.bounding_box = bounding_box

        self.mesh = chrono.ChTriangleMeshConnected()
        self.mesh.LoadWavefrontMesh(chrono.GetChronoDataFile(filename), False, True)
        self.shape = chrono.ChTriangleMeshShape()
        self.shape.SetMesh(self.mesh)
        self.shape.SetStatic(True)
        self.body = chrono.ChBody()
        self.body.AddAsset(self.shape)
        self.body.SetCollide(False)
        self.body.SetBodyFixed(True)

        self.scaled = False

        self.pos = chrono.ChVectorD()
        self.scale = 1
        self.ang = 0

    def UpdateCollisionModel(self, scale, z=5):
        self.body.SetCollide(True)
        size = self.bounding_box * scale / 2
        self.body.GetCollisionModel().ClearModel()
        self.body.GetCollisionModel().AddBox(size.x, size.y, z)
        self.body.GetCollisionModel().BuildModel()

    def CalcBoundingBox(self):
        """ Calculate approximate minimum boundary box of a mesh """
        vertices = self.mesh.m_vertices
        minimum = chrono.ChVectorD(min(vertices, key=lambda x: x.x).x, min(vertices, key=lambda x: x.y).y, 0)
        maximum = chrono.ChVectorD(max(vertices, key=lambda x: x.x).x, max(vertices, key=lambda x: x.y).y, 0)
        self.bounding_box = chrono.ChVectorD(maximum - minimum)

    def Scale(self, scale):
        if not self.scaled:
            self.scaled = True
            self.bounding_box *= scale

class Asset():
    def __init__(self, mesh, min_scale, max_scale, num=0):
        self.mesh = mesh
        self.min_scale = min_scale
        self.max_scale = max_scale
        self.num = num

        self.pos = chrono.ChVectorD()
        self.scale = 1
        self.ang = 0

        self.frames = sens.vector_ChFrameD()
        self.frames_list = []

    def Transform(self, pos, scale=1, ang=0):
        self.mesh.body.SetPos(pos)
        self.mesh.mesh.Transform(chrono.ChVectorD(0,0,0), chrono.ChMatrix33D(scale))
        self.mesh.body.SetRot(chrono.Q_from_AngAxis(ang, chrono.ChVectorD(0, 0, 1)))

        self.pos = pos
        self.scale = scale
        self.ang = ang
        self.rot = chrono.Q_from_AngAxis(ang, chrono.ChVectorD(0, 0, 1))
        
        self.mesh.Scale(scale)

    def GetContactForceLength(self):
        return self.mesh.body.GetContactForce().Length()

    def Clear(self):
        self.frames = sens.vector_ChFrameD()
        self.frame_list = []

class AssetList():
    def __init__(self, b1=0, b2=0, r1=0, r2=0, r3=0, r4=0, r5=0, t1=0, t2=0, t3=0, c=0):
        self.assets = []
        self.assets.append(Asset(AssetMesh("sensor/offroad/bush.obj", chrono.ChVectorD(1.35348, 1.33575, 0)), 0.5, 1.5, b1))
        self.assets.append(Asset(AssetMesh("sensor/offroad/bush2.obj", chrono.ChVectorD(3.21499, 3.30454, 0)), 0.5, 1.5, b2))
        self.assets.append(Asset(AssetMesh("sensor/offroad/rock1.obj", chrono.ChVectorD(3.18344, 3.62827, 0)), 0.25, 1, r1))
        self.assets.append(Asset(AssetMesh("sensor/offroad/rock2.obj", chrono.ChVectorD(4.01152, 2.64947, 0)), 0.25, .75, r2))
        self.assets.append(Asset(AssetMesh("sensor/offroad/rock3.obj", chrono.ChVectorD(2.53149, 2.48862, 0)), 0.25, .75, r3))
        self.assets.append(Asset(AssetMesh("sensor/offroad/rock4.obj", chrono.ChVectorD(2.4181, 4.47276, 0)), 0.25, .75, r4))
        self.assets.append(Asset(AssetMesh("sensor/offroad/rock5.obj", chrono.ChVectorD(3.80205, 2.56996, 0)), 0.25, .75, r5))
        self.assets.append(Asset(AssetMesh("sensor/offroad/tree1.obj", chrono.ChVectorD(2.39271, 2.36872, 0)), 0.5, 2, t1))
        self.assets.append(Asset(AssetMesh("sensor/offroad/tree2.obj", chrono.ChVectorD(9.13849, 8.7707, 0)), 0.15, .5, t2))
        self.assets.append(Asset(AssetMesh("sensor/offroad/tree3.obj", chrono.ChVectorD(4.7282, 4.67921, 0)), 5, 5, t3))
        self.assets.append(Asset(AssetMesh("sensor/offroad/cottage.obj", chrono.ChVectorD(33.9308, 20.7355, 0)), 1, 1, c))

        self.positions = []

    def Clear(self):
        self.positions = []
        for asset in self.assets:
            asset.Clear()

    def TransformAgain(self):
        """ Transform underlying mesh again since a sensor manager was created (Ask Asher) """
        for asset in self.assets:
            asset.Transform(asset.pos, asset.scale, asset.ang)

    def GenerateFrame(self, pos, ang, scale):
        # Calculate quaternion
        rot = chrono.Q_from_AngAxis(ang, chrono.ChVectorD(0, 0, 1))

        # Generate ChFrame which will then be scaled
        frame = chrono.ChFrameD(pos, rot)

        # Scale frame
        mat = frame.GetA().GetMatr()
        mat = [[x*scale for x in z] for z in mat]
        frame.GetA().SetMatr(mat)

        return frame

    def RandomlyPositionAssets(self, system, vehicle_pos, goal_pos, terrain, length, width):
        for asset in self.assets:
            for _ in range(asset.num):
                # Check if position is too close to another asset, vehicle or goal
                while True:
                    # Calculate random transformation values
                    pos = self.CalcRandomPose(terrain, length, width, offset=-random.random()*.5)
                    scale = self.map(random.random(), asset.min_scale, asset.max_scale)
                    threshold = asset.mesh.bounding_box.Length() * scale / 2
                    if len(self.positions) == 0:
                        break
                    min_pos = min(self.positions, key=lambda x: (x - pos).Length())
                    if (pos - vehicle_pos).Length() > 15 and (pos - min_pos).Length() > threshold and (pos - goal_pos).Length() > 15:
                        break

                # Calculate other random values
                ang = random.random()*chrono.CH_C_PI
                frame = self.GenerateFrame(pos, ang, scale)
                # scale = 10
                # asset.Transform(pos, scale, ang)

                self.positions.append(pos)
                asset.frames.append(frame)

                asset.mesh.pos = pos
                asset.mesh.ang = ang
                asset.mesh.scale = scale

                # Update the collision model
                # asset.mesh.UpdateCollisionModel(scale)

                # system.Add(asset.mesh.body)

    def map(self, value, min, max):
        """ Scale a random value to be within a range """
        return min + (value * (max - min))

    def CalcRandomPose(self, terrain, length, width, offset=0):
        """
        Calculates random position within the terrain boundaries

        TODO: generate some rotation (quaternion) to have mesh lay flush with the terrain
        """
        x = random.randint(-length/2, length/2)
        y = random.randint(-width/2, width/2)
        z = terrain.GetHeight(x, y) + offset
        return chrono.ChVectorD(x,y,z)

    def CalcContactForces(self, chassis_body, collision_box):
        pos = chassis_body.GetPos()
        for asset_pos in self.positions:
            # box1 = np.array([collision_box.x, collision_box.y])
            # box2 = np.array([asset.mesh.bounding_box.x, asset.mesh.bounding_box.y])
            # if areColliding(chassis_body, asset.mesh.body, box1, box2):
            #     return 1
            if (pos - asset_pos).Length() < 4:
                return 1
        return 0

    def GetNum(self):
        return len(self.assets)

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

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Tuple((
                spaces.Box(low=0, high=255, shape=(self.camera_height, self.camera_width, 3), dtype=np.uint8),  # camera
                spaces.Box(low=-100, high=100, shape=(7,), dtype=np.float)))                                        # goal gps

        self.info =  {"timeout": 10000.0}
        self.timestep = 3e-3

        # -------------------------------
        # Initialize simulation settings
        # -------------------------------

        self.timeend = 20
        self.control_frequency = 10

        self.min_terrain_height = 0     # min terrain height
        self.max_terrain_height = 0 # max terrain height
        self.terrain_length = 100.0 # size in X direction
        self.terrain_width = 100.0  # size in Y direction

        self.initLoc = chrono.ChVectorD(-self.terrain_length / 2.25, -self.terrain_width / 2.25, self.max_terrain_height + 1)
        self.initRot = chrono.ChQuaternionD(1, 0, 0, 0)

        b1 = 0
        b2 = 0
        r1 = 5
        r2 = 5
        r3 = 5
        r4 = 5
        r5 = 5
        t1 = 0
        t2 = 0
        t3 = 0
        c = 0
        self.assets = AssetList(b1, b2, r1, r2, r3, r4, r5, t1, t2, t3, c)

        self.render_setup = False
        self.play_mode = False

    def reset(self):
        # Create systems
        self.system = chrono.ChSystemNSC()
        self.system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
        self.system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
        self.system.SetSolverMaxIterations(150)
        self.system.SetMaxPenetrationRecoverySpeed(4.0)

        # Create the terrain
        self.bitmap_file =  os.path.dirname(os.path.realpath(__file__)) + "/utils/height_map.bmp"
        self.bitmap_file_backup =  os.path.dirname(os.path.realpath(__file__)) + "/utils/height_map_backup.bmp"
        shape = (252, 252)
        generate_random_bitmap(shape=shape, resolutions=[(18, 18), (12, 12), (2, 2)], mappings=[(-.1, -.1), (-.25,.25), (-1.5,1.5)], file_name=self.bitmap_file)
        self.terrain = veh.RigidTerrain(self.system)
        try:
            patch = self.terrain.AddPatch(chrono.CSYSNORM,       # position
                                        self.bitmap_file,        # heightmap file (.bmp)
                                        "test",                  # mesh name
                                        self.terrain_length*1.5,     # sizeX
                                        self.terrain_width*1.5,      # sizeY
                                        self.min_terrain_height, # hMin
                                        self.max_terrain_height) # hMax
        except Exception:
            print('Corrupt Bitmap File')
            patch = self.terrain.AddPatch(chrono.CSYSNORM,       # position
                                        self.bitmap_file_backup,        # heightmap file (.bmp)
                                        "test",                  # mesh name
                                        self.terrain_length*1.5,     # sizeX
                                        self.terrain_width*1.5,      # sizeY
                                        self.min_terrain_height, # hMin
                                        self.max_terrain_height) # hMax
        patch.SetTexture(chrono.GetChronoDataFile("sensor/textures/grass_texture.jpg"), 16, 16)

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
        vis_mat.SetKdTexture(chrono.GetChronoDataFile("sensor/textures/grass_texture.jpg"))
        vis_mat.SetSpecularColor(chrono.ChVectorF(0.9, 0.9, 0.9))
        vis_mat.SetFresnelMin(0)
        vis_mat.SetFresnelMax(0.01)
        visual_asset.material_list.append(vis_mat)

        self.initLoc.z = self.terrain.GetHeight(self.initLoc.x, self.initLoc.y) + 1.5

        self.vehicle = veh.HMMWV_Reduced(self.system)
        self.vehicle.SetContactMethod(chrono.ChMaterialSurface.NSC)
        self.vehicle.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
        self.vehicle.SetChassisFixed(False)
        self.vehicle.SetInitPosition(chrono.ChCoordsysD(self.initLoc, self.initRot))
        self.vehicle.SetTireType(veh.TireModelType_RIGID)
        self.vehicle.SetTireStepSize(self.timestep)
        self.vehicle.Initialize()

        if self.play_mode:
            self.vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
            self.vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
            self.vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)
        else:
            self.vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
            self.vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
            self.vehicle.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)
        self.vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
        self.vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
        self.chassis_body = self.vehicle.GetChassisBody()
        # self.chassis_body.GetCollisionModel().ClearModel()
        # size = chrono.ChVectorD(3,2,0.2)
        # self.chassis_body.GetCollisionModel().AddBox(0.5 * size.x, 0.5 * size.y, 0.5 * size.z)
        # self.chassis_body.GetCollisionModel().BuildModel()
        self.chassis_collision_box = chrono.ChVectorD(3,2,0.2)

        # Driver
        self.driver = veh.ChDriver(self.vehicle.GetVehicle())

        # create goal
        gx = random.random() * self.terrain_length - self.terrain_length / 2
        gy = random.random() * self.terrain_width - self.terrain_width / 2
        self.goal = chrono.ChVectorD(gx, gy, self.terrain.GetHeight(gx, gy) + 1)

        i = 0
        while (self.goal - self.initLoc).Length() < 15:
            gx = random.random() * self.terrain_length - self.terrain_length / 2
            gy = random.random() * self.terrain_width - self.terrain_width / 2
            self.goal = chrono.ChVectorD(gx, gy, self.max_terrain_height + 1)
            if i > 100:
                print('Break')
                break
            i += 1
        self.goal_coord = toGPSCoordinate(self.goal)
        self.origin = GPSCoord(43.073268, -89.400636, 260.0)

        self.goal_sphere = chrono.ChBodyEasySphere(.25, 1000, False, True)
        self.goal_sphere.SetBodyFixed(True)
        self.goal_sphere.AddAsset(chrono.ChColorAsset(1,0,0))
        self.goal_sphere.SetPos(self.goal)
        if self.play_mode:
            self.system.Add(self.goal_sphere)

        # create obstacles
        # start = t.time()
        self.assets.Clear()
        self.assets.RandomlyPositionAssets(self.system, self.initLoc, self.goal, self.terrain, self.terrain_length*1.5, self.terrain_width*1.5)
        # print('Assets Add :: ', t.time() - start)

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
        intensity = 0.75
        self.manager.scene.AddPointLight(chrono.ChVectorF(100, 100, 100), chrono.ChVectorF(intensity, intensity, intensity), 5000.0)
        self.manager.scene.AddPointLight(chrono.ChVectorF(-100, -100, 100), chrono.ChVectorF(intensity, intensity, intensity), 5000.0)

        if self.play_mode:
            self.manager.scene.GetBackground().has_texture = True;
            self.manager.scene.GetBackground().env_tex = "sensor/textures/qwantani_8k.hdr"
            self.manager.scene.GetBackground().has_changed = True;
        # -----------------------------------------------------
        # Create a self.camera and add it to the sensor manager
        # -----------------------------------------------------
        self.camera = sens.ChCameraSensor(
            self.chassis_body,  # body camera is attached to
            50,  # scanning rate in Hz
            chrono.ChFrameD(chrono.ChVectorD(1.5, 0, .875), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))),
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
            origin,
            gps_noise_none
        )
        self.gps.SetName("GPS Sensor")
        self.gps.FilterList().append(sens.ChFilterGPSAccess())
        self.manager.AddSensor(self.gps)

        # have to reconstruct scene because sensor loads in meshes separately (ask Asher)
        # start = t.time()
        if self.assets.GetNum() > 0:
            # self.assets.TransformAgain()
            # start = t.time()
            for asset in self.assets.assets:
                if len(asset.frames) > 0:
                    self.manager.AddInstancedStaticSceneMeshes(asset.frames, asset.mesh.shape)
            # self.manager.ReconstructScenes()
            # self.manager.AddInstancedStaticSceneMeshes(self.assets.frames, self.assets.shapes)
            # self.manager.Update()
            # print('Reconstruction :: ', t.time() - start)

        self.old_dist = (self.goal - self.initLoc).Length()

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
            # start = t.time()
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
                braking = np.clip(abs(self.ac[1,]), self.driver.GetBraking() - self.BrakingDelta, self.driver.GetBraking() + self.BrakingDelta)
                throttle = np.clip(0, self.driver.GetThrottle() - self.ThrottleDelta, self.driver.GetThrottle() + self.ThrottleDelta)

            self.driver.SetSteering(steering)
            self.driver.SetThrottle(throttle)
            self.driver.SetBraking(braking)

            # Advance simulation for one timestep for all modules
            self.driver.Advance(self.timestep)
            self.vehicle.Advance(self.timestep)
            self.terrain.Advance(self.timestep)
            self.system.DoStepDynamics(self.timestep)
            # chrono_time = t.time() - start
            # sens_start = t.time()
            self.manager.Update()
            # sensor_time = t.time() - sens_start
            # if sensor_time > 1e-4:
                # print('Chrono :: ', chrono_time)
                # print('Sensor :: ', sensor_time)

            self.c_f += self.assets.CalcContactForces(self.chassis_body, self.chassis_collision_box)
            if self.c_f:
                break
        
        self.rew = self.calc_rew()
        self.obs = self.get_ob()
        self.is_done()
        return self.obs, self.rew, self.isdone, self.info

    def get_ob(self):
        camera_buffer_RGBA8 = self.camera.GetMostRecentRGBA8Buffer()
        if camera_buffer_RGBA8.HasData():
            rgb = camera_buffer_RGBA8.GetRGBA8Data()[:,:,0:3]
        else:
            rgb = np.zeros((self.camera_height,self.camera_width,3))
            # print('NO DATA \n')

        gps_buffer = self.gps.GetMostRecentGPSBuffer()
        if gps_buffer.HasData():
            cur_gps_data = gps_buffer.GetGPSData()[0:3]
            cur_gps_data = GPSCoord(cur_gps_data[1], cur_gps_data[0], cur_gps_data[2])
        else:
            cur_gps_data = GPSCoord(origin.lat, origin.long)

        # goal_gps_data = np.array([self.goal_coord.x, self.goal_coord.y, self.goal_coord.z])

        # err = self.goal - self.chassis_body.GetPos()
        pos = self.chassis_body.GetPos()
        vel = self.vehicle.GetChassisBody().GetFrame_REF_to_abs().GetPos_dt()
        # goal_gps_data = np.array([self.goal.x, self.goal.y, pos.x, pos.y, vel.x, vel.y])
        gps_data = (self.goal_coord - cur_gps_data)
        gps_data = np.array([gps_data.x, gps_data.y]) * 10000
        # gps_data = np.array([gps_data[0], gps_data[1], vel.x, vel.y])
        # print(cur_gps_data, self.origin)
        sens.GPS2Cartesian(cur_gps_data, self.origin)
        # print(pos, cur_gps_data)
        # pos_data = [self.goal.x, self.goal.y, cur_gps_data.x, cur_gps_data.y, vel.x, vel.y]
        vec_obs = np.asarray([self.goal.x, self.goal.y, pos.x, pos.y,self.chassis_body.GetRot().Q_to_Euler123().z, vel.x, vel.y])
        return (rgb, vec_obs)

    def calc_rew(self):
        progress_coeff = 20
        vel_coeff = 0
        time_cost = 0
        progress = self.calc_progress()
        # vel = self.vehicle.GetVehicle().GetVehicleSpeed()
        rew = progress_coeff*progress# + vel_coeff*vel + time_cost*self.system.GetChTime()
        return rew

    def is_done(self):

        pos = self.chassis_body.GetPos()

        collision = not(self.c_f == 0)
        if self.system.GetChTime() > self.timeend:
            dist = (pos - self.goal).Length()
            print('Timeout!! Distance from goal :: ', dist)
            self.isdone = True
            self.rew -= 250
            failed = 0
        elif abs(pos.x) > self.terrain_length * 1.5 / 2.0 or abs(pos.y) > self.terrain_width * 1.5 / 2 or pos.z < self.min_terrain_height:
            dist = (self.chassis_body.GetPos() - self.goal).Length()
            print('Fell off terrain!! Distance from goal :: ', dist)
            #self.rew -= 250
            self.isdone = True
            failed = 1
        elif collision:
            #self.rew -= 250
            print('Hit object!!')
            self.isdone = True
            failed = 2
        elif (pos - self.goal).Length() < 5:
            self.rew += 5000
            print('Success!!')
            # self.successes += 1
            self.isdone = True
            failed = 3

        if self.isdone:
            goal = np.array([self.goal.x, self.goal.y])
            from csv import writer
            with open('./Monitor/log.csv', 'a+', newline='') as write_obj:
                # Timeout = 0, fell off = 1, obstacle = 2, success = 3
                goal = [failed, goal[0], goal[1]]
                csv_writer = writer(write_obj)
                csv_writer.writerow(goal)

    def calc_progress(self):
        dist = (self.chassis_body.GetPos() - self.goal).Length()
        progress = self.old_dist - dist
        # print(dist, self.old_dist)
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
                width = 600
                height = 400
                vis_camera = sens.ChCameraSensor(
                    self.chassis_body,  # body camera is attached to
                    30,  # scanning rate in Hz
                    chrono.ChFrameD(chrono.ChVectorD(-8, 0, 3), chrono.Q_from_AngAxis(chrono.CH_C_PI / 20, chrono.ChVectorD(0, 1, 0))),
                    # offset pose
                    width,  # number of horizontal samples
                    height,  # number of vertical channels
                    chrono.CH_C_PI / 3,  # horizontal field of view
                    (height/width) * chrono.CH_C_PI / 3.  # vertical field of view
                )
                vis_camera.SetName("Follow Camera Sensor")
                # self.camera.FilterList().append(sens.ChFilterVisualize(self.camera_width, self.camera_height, "RGB Camera"))
                # vis_camera.FilterList().append(sens.ChFilterVisualize(1280, 720, "Visualization Camera"))
                if True:
                    # vis_camera.FilterList().append(sens.ChFilterSave())
                    self.camera.FilterList().append(sens.ChFilterSave())
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
