# PyChrono imports
import pychrono as chrono
import pychrono.vehicle as veh
try:
   import pychrono.sensor as sens
except:
   print('Could not import Chrono Sensor')

# Default lib imports
import numpy as np
import os
import random

# Custom imports
from gym_chrono.envs.ChronoBase import ChronoBaseEnv
from gym_chrono.envs.utils.perlin_bitmap_generator import generate_random_bitmap
from gym_chrono.envs.utils.utilities import SetChronoDataDirectories, CalcInitialPose, areColliding
from gym_chrono.envs.utils.SCM_parameters import *
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
        self.mesh.mesh.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(scale))
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
        self.assets.append(
            Asset(AssetMesh("sensor/offroad/bush1.obj", chrono.ChVectorD(1.35348, 1.33575, 0)), 0.5, 1.5, b1))
        self.assets.append(
            Asset(AssetMesh("sensor/offroad/bush2.obj", chrono.ChVectorD(3.21499, 3.30454, 0)), 0.5, 1.5, b2))
        self.assets.append(
            Asset(AssetMesh("sensor/offroad/rock1.obj", chrono.ChVectorD(3.18344, 3.62827, 0)), 0.25, 1, r1))
        self.assets.append(
            Asset(AssetMesh("sensor/offroad/rock2.obj", chrono.ChVectorD(4.01152, 2.64947, 0)), 0.25, .75, r2))
        self.assets.append(
            Asset(AssetMesh("sensor/offroad/rock3.obj", chrono.ChVectorD(2.53149, 2.48862, 0)), 0.25, .75, r3))
        self.assets.append(
            Asset(AssetMesh("sensor/offroad/rock4.obj", chrono.ChVectorD(2.4181, 4.47276, 0)), 0.25, .75, r4))
        self.assets.append(
            Asset(AssetMesh("sensor/offroad/rock5.obj", chrono.ChVectorD(3.80205, 2.56996, 0)), 0.25, .75, r5))
        self.assets.append(
            Asset(AssetMesh("sensor/offroad/tree1.obj", chrono.ChVectorD(2.39271, 2.36872, 0)), 0.5, 2, t1))
        self.assets.append(
            Asset(AssetMesh("sensor/offroad/tree2.obj", chrono.ChVectorD(9.13849, 8.7707, 0)), 0.15, .5, t2))
        self.assets.append(Asset(AssetMesh("sensor/offroad/tree3.obj", chrono.ChVectorD(4.7282, 4.67921, 0)), 5, 5, t3))
        self.assets.append(
            Asset(AssetMesh("sensor/offroad/cottage.obj", chrono.ChVectorD(33.9308, 20.7355, 0)), 1, 1, c))

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
        mat = [[x * scale for x in z] for z in mat]
        frame.GetA().SetMatr(mat)

        return frame

    def RandomlyPositionAssets(self, system, vehicle_pos, goal_pos, terrain, length, width, should_scale=True):
        for asset in self.assets:
            for _ in range(asset.num):
                # Check if position is too close to another asset, vehicle or goal
                while True:
                    # Calculate random transformation values
                    pos = self.CalcRandomPose(terrain, length, width, offset=-random.random() * .5)
                    if should_scale:
                        scale = self.map(random.random(), asset.min_scale, asset.max_scale)
                    else:
                        scale = 1
                    threshold = asset.mesh.bounding_box.Length() * scale / 2
                    if len(self.positions) == 0:
                        break
                    min_pos = min(self.positions, key=lambda x: (x - pos).Length())
                    if (pos - vehicle_pos).Length() > 15 and (pos - min_pos).Length() > threshold and (
                            pos - goal_pos).Length() > 15:
                        break

                # Calculate other random values
                ang = random.random() * chrono.CH_C_PI
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
        x = random.randint(int(-length / 2), int(length / 2))
        y = random.randint(int(-width / 2), int(width / 2))
        z = terrain.GetHeight(chrono.ChVectorD(x, y, 0)) + offset
        return chrono.ChVectorD(x, y, z)

    def CalcContactForces(self, chassis_body, collision_box):
        pos = chassis_body.GetPos()
        for asset_pos in self.positions:
            # box1 = np.array([collision_box.x, collision_box.y])
            # box2 = np.array([asset.mesh.bounding_box.x, asset.mesh.bounding_box.y])
            # if areColliding(chassis_body, asset.mesh.body, box1, box2):
            #     return 1
            if (pos - asset_pos).Length() < 3:
                return 1
        return 0

    def GetClosestAssetDist(self, chassis_body, min_dist=200):
        pos = chassis_body.GetPos()
        for asset_pos in self.positions:
            dist = (pos - asset_pos).Length()
            if dist < min_dist:
                min_dist = dist
        return min_dist

    def GetNum(self):
        return len(self.assets)


class off_road_gator_v3(ChronoBaseEnv):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self):
        ChronoBaseEnv.__init__(self)

        # Set Chrono data directories
        SetChronoDataDirectories()

        # Define action and observation space
        # They must be gym.spaces objects
        # Example when using discrete actions:
        self.camera_width = 80
        self.camera_height = 45

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Tuple((
            spaces.Box(low=0, high=255, shape=(self.camera_height, self.camera_width, 3), dtype=np.uint8),  # camera
            spaces.Box(low=-100, high=100, shape=(5,), dtype=np.float)))  # goal gps
        # self.observation_space = spaces.Box(low=-100, high=100, shape=(6,), dtype=np.float)

        self.info = {"timeout": 10000.0}
        self.timestep = 2e-3

        # -------------------------------
        # Initialize simulation settings
        # -------------------------------

        self.timeend = 20
        self.control_frequency = 10

        self.min_terrain_height = -5  # min terrain height
        self.max_terrain_height = 5  # max terrain height
        self.terrain_length = 80.0  # size in X direction
        self.terrain_width = 80.0  # size in Y direction

        self.render_setup = False
        self.play_mode = False

    def reset(self):
        n = 10
        b1 = 0
        b2 = 0
        r1 = n
        r2 = n
        r3 = n
        r4 = n
        r5 = n
        t1 = 0
        t2 = 0
        t3 = 0
        c = 0
        self.assets = AssetList(b1, b2, r1, r2, r3, r4, r5, t1, t2, t3, c)
        # Create systems
        self.system = chrono.ChSystemNSC()
        self.system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
        self.system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
        self.system.SetSolverMaxIterations(150)
        self.system.SetMaxPenetrationRecoverySpeed(4.0)

        # Create the terrain
        rigid_terrain = False
        # Ghost Terrain: rigid, vis, no contact
        self.ghost_terrain = veh.RigidTerrain(self.system)
        patch_mat = chrono.ChMaterialSurfaceNSC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
        # Real Terrain: deformable, no vis, contact
        self.terrain = veh.SCMDeformableTerrain(self.system, False)
        # Parameters
        randpar = random.randint(1, 6)
        params = SCMParameters()
        params.InitializeParametersAsHard()
        params.SetParameters(self.terrain)
        # Bulldozing
        self.terrain.EnableBulldozing(True)
        self.terrain.SetBulldozingParameters(
            55,  # angle of friction for erosion of displaced material at the border of the rut
            1,  # displaced material vs downward pressed material.
            5,  # number of erosion refinements per timestep
            10)  # number of concentric vertex selections subject to erosion

        self.bitmap_file = os.path.dirname(os.path.realpath(__file__)) + "/../../utils/height_map.bmp"
        self.bitmap_file_backup = os.path.dirname(os.path.realpath(__file__)) + "/../../utils/height_map_backup.bmp"
        shape = (252, 252)
        generate_random_bitmap(shape=shape, resolutions=[(2, 2)], mappings=[(-1.5, 1.5)], file_name=self.bitmap_file)
        try:
            patch = self.ghost_terrain.AddPatch(patch_mat,
                                          chrono.CSYSNORM,  # position
                                          self.bitmap_file,  # heightmap file (.bmp)
                                          "test",  # mesh name
                                          self.terrain_length * 1.5,  # sizeX
                                          self.terrain_width * 1.5,  # sizeY
                                          self.min_terrain_height,  # hMin
                                          self.max_terrain_height)  # hMax
            self.terrain.Initialize(self.bitmap_file,  # heightmap file (.bmp)
                                    self.terrain_length * 1.5,  # sizeX
                                    self.terrain_width * 1.5,  # sizeY
                                    self.min_terrain_height,  # hMin
                                    self.max_terrain_height,  # hMax
                                    0.05)  # Delta

        except Exception:
            print('Corrupt Bitmap File')
            patch = self.ghost_terrain.AddPatch(patch_mat,
                                          chrono.CSYSNORM,  # position
                                          self.bitmap_file_backup,  # heightmap file (.bmp)
                                          "test",  # mesh name
                                          self.terrain_length * 1.5,  # sizeX
                                          self.terrain_width * 1.5,  # sizeY
                                          self.min_terrain_height,  # hMin
                                          self.max_terrain_height)  # hMax
            self.terrain.Initialize(self.bitmap_file_backup,  # heightmap file (.bmp)
                                    self.terrain_length * 1.5,  # sizeX
                                    self.terrain_width * 1.5,  # sizeY
                                    self.min_terrain_height,  # hMin
                                    self.max_terrain_height,  # hMax
                                    0.05)

        patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
        patch.SetColor(chrono.ChColor(1.0, 1.0, 1.0))
        self.ghost_terrain.Initialize()
        ground_body = patch.GetGroundBody()
        ground_body.SetCollide(False)
        ground_asset = ground_body.GetAssets()[0]
        visual_asset = chrono.CastToChVisualization(ground_asset)
        visual_asset.SetStatic(True)
        vis_mat = chrono.ChVisualMaterial()
        tex = np.random.randint(1,6)
        if tex%2 == 0 and tex%5 != 0 :
            vis_mat.SetKdTexture(veh.GetDataFile("terrain/textures/grass.jpg"))
        elif tex%2 == 1 and tex%5 != 0 :
            vis_mat.SetKdTexture(chrono.GetChronoDataFile("sensor/textures/grass_texture.jpg"))
        visual_asset.material_list.append(vis_mat)

        theta = random.random() * 2 * np.pi
        x, y = self.terrain_length * 0.5 * np.cos(theta), self.terrain_width * 0.5 * np.sin(theta)
        z = self.terrain.GetHeight(chrono.ChVectorD(x, y, 0)) + 0.25
        ang = np.pi + theta
        self.initLoc = chrono.ChVectorD(x, y, z)
        self.initRot = chrono.Q_from_AngZ(ang)

        self.vehicle = veh.Gator(self.system)
        self.vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
        self.vehicle.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
        self.vehicle.SetChassisFixed(False)
        self.m_inputs = veh.Inputs()
        self.vehicle.SetInitPosition(chrono.ChCoordsysD(self.initLoc, self.initRot))
        self.vehicle.SetTireType(veh.TireModelType_RIGID_MESH)
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
        self.chassis_collision_box = chrono.ChVectorD(3, 2, 0.2)

        # Driver
        self.driver = veh.ChDriver(self.vehicle.GetVehicle())

        # create goal
        # pi/4 ang displ
        delta_theta = (random.random() - 0.5) * 1.0 * np.pi
        gx, gy = self.terrain_length * 0.5 * np.cos(theta + np.pi + delta_theta), self.terrain_width * 0.5 * np.sin(
            theta + np.pi + delta_theta)
        self.goal = chrono.ChVectorD(gx, gy, self.terrain.GetHeight(chrono.ChVectorD(gx, gy, 0)) + 1.0)

        i = 0
        while (self.goal - self.initLoc).Length() < 15:
            gx = random.random() * self.terrain_length - self.terrain_length / 2
            gy = random.random() * self.terrain_width - self.terrain_width / 2
            self.goal = chrono.ChVectorD(gx, gy, self.max_terrain_height + 1)
            if i > 100:
                print('Break')
                break
            i += 1

        # self.goal = chrono.ChVectorD(75, 0, 0)
        # Origin in Madison WI
        self.origin = chrono.ChVectorD(43.073268, -89.400636, 260.0)
        self.goal_coord = chrono.ChVectorD(self.goal)
        sens.Cartesian2GPS(self.goal_coord, self.origin)

        self.goal_sphere = chrono.ChBodyEasySphere(.55, 1000, True, False)
        self.goal_sphere.SetBodyFixed(True)

        sphere_asset = self.goal_sphere.GetAssets()[0]
        visual_asset = chrono.CastToChVisualization(sphere_asset)

        vis_mat = chrono.ChVisualMaterial()
        vis_mat.SetAmbientColor(chrono.ChVectorF(0, 0, 0))
        vis_mat.SetDiffuseColor(chrono.ChVectorF(.2, .2, .9))
        vis_mat.SetSpecularColor(chrono.ChVectorF(.9, .9, .9))

        visual_asset.material_list.append(vis_mat)
        visual_asset.SetStatic(True)

        self.goal_sphere.SetPos(self.goal)
        if self.play_mode:
            self.system.Add(self.goal_sphere)

        # create obstacles
        # start = t.time()
        self.assets.Clear()
        self.assets.RandomlyPositionAssets(self.system, self.initLoc, self.goal, self.terrain,
                                           self.terrain_length * 1.5, self.terrain_width * 1.5, should_scale=False)

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
        self.manager.scene.AddPointLight(chrono.ChVectorF(100, 100, 100), chrono.ChVectorF(1, 1, 1), 5000.0)
        self.manager.scene.AddPointLight(chrono.ChVectorF(-100, -100, 100), chrono.ChVectorF(1, 1, 1), 5000.0)
        # Let's not, for the moment, give a different scenario during test
        """
        if self.play_mode:
            self.manager.scene.GetBackground().has_texture = True;
            self.manager.scene.GetBackground().env_tex = "sensor/textures/qwantani_8k.hdr"
            self.manager.scene.GetBackground().has_changed = True;
        """
        # -----------------------------------------------------
        # Create a self.camera and add it to the sensor manager
        # -----------------------------------------------------
        #chrono.ChFrameD(chrono.ChVectorD(1.5, 0, .875), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))),
        self.camera = sens.ChCameraSensor(
            self.chassis_body,  # body camera is attached to
            20,  # scanning rate in Hz
            chrono.ChFrameD(chrono.ChVectorD(.65, 0, .75), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))),
            # offset pose
            self.camera_width,  # number of horizontal samples
            self.camera_height,  # number of vertical channels
            chrono.CH_C_PI / 2,  # horizontal field of view
            6  # supersampling factor
        )
        self.camera.SetName("Camera Sensor")
        self.camera.PushFilter(sens.ChFilterRGBA8Access())
        #if self.play_mode:
        #    self.camera.PushFilter(sens.ChFilterVisualize(self.camera_width, self.camera_height, "RGB Camera"))
        self.manager.AddSensor(self.camera)

        # -----------------------------------------------------
        # Create a self.gps and add it to the sensor manager
        # -----------------------------------------------------
        gps_noise_none = sens.ChGPSNoiseNone()
        self.gps = sens.ChGPSSensor(
            self.chassis_body,
            15,
            chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))),
            self.origin,
            gps_noise_none
        )
        self.gps.SetName("GPS Sensor")
        self.gps.PushFilter(sens.ChFilterGPSAccess())
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

        self.old_dist = (self.goal - self.initLoc).Length()

        self.step_number = 0
        self.c_f = 0
        self.old_ac = 0
        self.isdone = False
        self.render_setup = False
        self.dist0 = (self.goal - self.chassis_body.GetPos()).Length()
        if self.play_mode:
            self.render()

        return self.get_ob()

    def step(self, ac):
        self.ac = ac.reshape((-1,))
        # Collect output data from modules (for inter-module communication)

        for i in range(round(1 / (self.control_frequency * self.timestep))):
            # start = t.time()
            time = self.system.GetChTime()

            self.m_inputs.m_steering = np.clip(self.ac[0,], self.m_inputs.m_steering - self.SteeringDelta,
                                               self.m_inputs.m_steering + self.SteeringDelta)
            if self.ac[1,] > 0:
                self.m_inputs.m_throttle = np.clip(abs(self.ac[1,]), self.m_inputs.m_throttle - self.ThrottleDelta,
                                                   self.m_inputs.m_throttle + self.ThrottleDelta)
                self.m_inputs.m_braking = np.clip(0, self.m_inputs.m_braking - self.BrakingDelta,
                                                  self.m_inputs.m_braking + self.BrakingDelta)
            else:
                self.m_inputs.m_braking = np.clip(abs(self.ac[1,]), self.m_inputs.m_braking - self.BrakingDelta,
                                                  self.m_inputs.m_braking + self.BrakingDelta)
                self.m_inputs.m_throttle = np.clip(0, self.m_inputs.m_throttle - self.ThrottleDelta,
                                                   self.m_inputs.m_throttle + self.ThrottleDelta)
            # self.driver.Synchronize(time)
            self.vehicle.Synchronize(time, self.m_inputs, self.terrain)
            self.terrain.Synchronize(time)

            # Advance simulation for one timestep for all modules
            self.driver.Advance(self.timestep)
            self.vehicle.Advance(self.timestep)
            self.terrain.Advance(self.timestep)
            self.system.DoStepDynamics(self.timestep)
            # chrono_time = t.time() - start
            # sens_start = t.time()
            self.manager.Update()


            self.c_f += self.assets.CalcContactForces(self.chassis_body, self.chassis_collision_box)
            if self.c_f:
                break
            # for box in self.boxes:
            #     self.c_f += box.GetContactForce().Length()

        self.rew = self.calc_rew()
        self.obs = self.get_ob()
        self.is_done()
        return self.obs, self.rew, self.isdone, self.info

    def get_ob(self):
        camera_buffer_RGBA8 = self.camera.GetMostRecentRGBA8Buffer()
        if camera_buffer_RGBA8.HasData():
            rgb = camera_buffer_RGBA8.GetRGBA8Data()[:, :, 0:3]
        else:
            rgb = np.zeros((self.camera_height, self.camera_width, 3))
        # rgb = np.zeros((self.camera_height,self.camera_width,3))

        gps_buffer = self.gps.GetMostRecentGPSBuffer()
        if gps_buffer.HasData():
            cur_gps_data = gps_buffer.GetGPSData()
            cur_gps_data = chrono.ChVectorD(cur_gps_data[1], cur_gps_data[0], cur_gps_data[2])
        else:
            cur_gps_data = chrono.ChVectorD(self.origin)

        pos = self.chassis_body.GetPos()
        vel = self.chassis_body.GetPos_dt()

        head = self.vehicle.GetVehicle().GetVehicleRot().Q_to_Euler123().z
        gps_data = [(self.goal - self.chassis_body.GetPos()).x, (self.goal - self.chassis_body.GetPos()).y]
        dist = self.goal - self.chassis_body.GetPos()
        dist_local = self.chassis_body.GetRot().RotateBack(dist)
        targ_head = np.arctan2(dist_local.y, dist_local.x)
        goalCart = chrono.ChVectorD(self.goal_coord)
        sens.GPS2Cartesian(goalCart, self.origin)
        sens.GPS2Cartesian(cur_gps_data, self.origin)
        gps_dist = goalCart - cur_gps_data
        loc_dist_gps = [gps_dist.x * np.cos(head) + gps_dist.y * np.sin(head),
                        -gps_dist.x * np.sin(head) + gps_dist.y * np.cos(head)]
        array_data = np.array([loc_dist_gps[0], loc_dist_gps[1], head, targ_head, vel.Length()])
        return (rgb, array_data)

    def calc_rew(self):
        progress_coeff = 20
        # vel_coeff = .01
        time_cost = 0
        progress = self.calc_progress()
        deltaac = np.linalg.norm(self.ac - self.old_ac)
        self.old_ac = self.ac
        return 1.0*progress - 0.5*deltaac

    def is_done(self):

        pos = self.chassis_body.GetPos()

        collision = not (self.c_f == 0)
        if self.system.GetChTime() > self.timeend:
            dist = (pos - self.goal).Length()
            print('Timeout!! Distance from goal :: ', dist)
            self.isdone = True
            self.rew -= 400
            failed = 0
        elif abs(pos.x) > self.terrain_length * 1.5 / 2.0 or abs(
                pos.y) > self.terrain_width * 1.5 / 2 or pos.z < self.min_terrain_height:
            dist = (self.chassis_body.GetPos() - self.goal).Length()
            print('Fell off terrain!! Distance from goal :: ', dist)
            self.rew -= 250
            self.isdone = True
            failed = 1
        elif collision:
            self.rew -= 250
            print('Hit object')
            self.isdone = True
            failed = 2
        # elif abs(self.head_diff)>np.pi/2 :
        #    #self.rew -= 10000
        #    print('Out of trajectory')
        #    self.isdone = True
        #    failed = 2
        elif (pos - self.goal).Length() < 10:
            self.rew += 2500
            print('Success!!')
            # self.successes += 1
            self.isdone = True
            failed = 3

        # if self.isdone:
        #     goal = np.array([self.goal.x, self.goal.y])
        #     from csv import writer
        #     with open('./Monitor/log.csv', 'a+', newline='') as write_obj:
        #         # Timeout = 0, fell off = 1, obstacle = 2, success = 3
        #         goal = [failed, goal[0], goal[1]]
        #         csv_writer = writer(write_obj)
        #         csv_writer.writerow(goal)

    def calc_progress(self):
        dist = (self.chassis_body.GetPos() - self.goal).Length()
        progress = self.old_dist - dist
        self.old_dist = dist
        return progress

    def render(self, mode='human'):
        if not (self.play_mode == True):
            raise Exception('Please set play_mode=True to render')

        if not self.render_setup:
            vis = True
            save = False
            birds_eye = False
            third_person = True
            width = 1280
            height = 720
            if birds_eye:
                body = chrono.ChBodyAuxRef()
                body.SetBodyFixed(True)
                self.system.AddBody(body)
                vis_camera = sens.ChCameraSensor(
                    body,  # body camera is attached to
                    20,  # scanning rate in Hz
                    chrono.ChFrameD(chrono.ChVectorD(0, 0, 200),
                                    chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 1, 0))),
                    # offset pose
                    width,  # number of horizontal samples
                    height,  # number of vertical channels
                    chrono.CH_C_PI / 3  # horizontal field of view
                )
                vis_camera.SetName("Birds Eye Camera Sensor")
                if vis:
                    vis_camera.PushFilter(sens.ChFilterVisualize(width, height, "Visualization Camera"))
                if save:
                    vis_camera.PushFilter(sens.ChFilterSave())
                self.manager.AddSensor(vis_camera)

            if third_person:
                vis_camera = sens.ChCameraSensor(
                    self.chassis_body,  # body camera is attached to
                    20,  # scanning rate in Hz
                    chrono.ChFrameD(chrono.ChVectorD(-8, 0, 3),
                                    chrono.Q_from_AngAxis(chrono.CH_C_PI / 20, chrono.ChVectorD(0, 1, 0))),
                    # offset pose
                    width,  # number of horizontal samples
                    height,  # number of vertical channels
                    chrono.CH_C_PI / 3  # horizontal field of view
                )
                vis_camera.SetName("Follow Camera Sensor")
                if vis:
                    vis_camera.PushFilter(sens.ChFilterVisualize(width, height, "Visualization Camera"))
                if save:
                    vis_camera.PushFilter(sens.ChFilterSave())
                self.manager.AddSensor(vis_camera)

            # -----------------------------------------------------------------
            # Create a filter graph for post-processing the data from the lidar
            # -----------------------------------------------------------------

            # self.camera.PushFilter(sens.ChFilterVisualize("RGB Camera"))
            # vis_camera.PushFilter(sens.ChFilterVisualize("Visualization Camera"))
            self.render_setup = True

        if (mode == 'rgb_array'):
            return self.get_ob()

    def close(self):
        del self

    def ScreenCapture(self, interval):
        raise NotImplementedError

    def __del__(self):
        del self.manager
