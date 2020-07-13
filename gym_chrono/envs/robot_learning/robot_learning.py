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
from gym_chrono.envs.robot_learning.assets import *
from gym_chrono.envs.robot_learning.parameters import *

# openai-gym imports
import gym
from gym import spaces
import time as t

class robot_learning(ChronoBaseEnv):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}
    def __init__(self):
        ChronoBaseEnv.__init__(self)

        # Set Chrono data directories
        SetChronoDataDirectories()

        # Define action and observation space
        # They must be gym.spaces objects
        # Example when using discrete actions:
        self.camera_width  = 80
        self.camera_height = 45

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Tuple((
                spaces.Box(low=0, high=255, shape=(self.camera_height, self.camera_width, 3), dtype=np.uint8),  # camera
                spaces.Box(low=-100, high=100, shape=(5,), dtype=np.float)))                                        # goal gps
        #self.observation_space = spaces.Box(low=-100, high=100, shape=(6,), dtype=np.float)

        self.info =  {"timeout": 10000.0}
        self.timestep = 2e-3

        # -------------------------------
        # Initialize simulation settings
        # -------------------------------

        self.timeend = 40
        self.control_frequency = 10

        self.min_terrain_height = 0     # min terrain height
        self.max_terrain_height = 10 # max terrain height
        self.terrain_length = 80.0 # size in X direction
        self.terrain_width = 80.0  # size in Y direction
        self.divs_per_units = 20 # divisions per unit (SCM only)
        self.bitmap_img_size = tuple(np.array(np.array([self.terrain_length, self.terrain_width]) * self.divs_per_units, dtype=int))

        temp_dir = os.path.dirname(os.path.realpath(__file__))
        temp_dir = os.path.abspath(os.path.join(temp_dir, '..'))
        self.bitmap_file = temp_dir  + "/utils/height_map.png"

        self.render_setup = False
        self.play_mode = False

    def reset(self):
        # Specify the terrain type
        terrain_type = ''
        terrain_type += 'rigid'
        # terrain_type += 'scm_hard'
        # terrain_type += 'scm_soft'
        terrain_type += '_flat'
        # terrain_type += '_height_map'

        # Add objects
        seed = 3
        np.random.seed(seed)
        random.seed(seed)

        n = 8
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
        if 'soft' in terrain_type: b1 = b2 = t1 = t2 = t3 = 0
        self.assets = AssetList(b1, b2, r1, r2, r3, r4, r5, t1, t2, t3, c)

        # Create systems
        self.system = chrono.ChSystemNSC()
        self.system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
        self.system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
        self.system.SetSolverMaxIterations(50)
        self.system.SetMaxPenetrationRecoverySpeed(4.0)

        theta = random.random()*2*np.pi
        x,y = self.terrain_length*0.5*np.cos(theta) , self.terrain_width*0.5*np.sin(theta)
        ang = np.pi + theta
        x,y = -35,35
        self.initLoc = chrono.ChVectorD(x, y, 0)
        self.initRot = chrono.Q_from_AngZ(ang)

        # Create terrain
        if 'height_map' in terrain_type:
            shape = (252, 252)
            generate_random_bitmap(shape=shape, resolutions=[(2, 2)], mappings=[(-1.5,1.5)], img_size=(400,400), file_name=self.bitmap_file, initPos=self.initLoc)

        if 'rigid' in terrain_type:
            self.terrain = veh.RigidTerrain(self.system)

            patch_mat = chrono.ChMaterialSurfaceNSC()
            patch_mat.SetFriction(0.9)
            patch_mat.SetRestitution(0.01)

            if 'flat' in terrain_type:
                patch = self.terrain.AddPatch(patch_mat,
                                         chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1),
                                         self.terrain_length, self.terrain_width)
            elif 'height_map' in terrain_type:
                patch = self.terrain.AddPatch(patch_mat,
                                            chrono.CSYSNORM,       # position
                                            self.bitmap_file,        # heightmap file (.bmp)
                                            "test",                  # mesh name
                                            self.terrain_length,     # sizeX
                                            self.terrain_width,      # sizeY
                                            self.min_terrain_height, # hMin
                                            self.max_terrain_height) # hMax

            self.terrain.Initialize()

            texture_file = chrono.GetChronoDataFile("sensor/textures/grass_texture.jpg")
            # texture_file = veh.GetDataFile("terrain/textures/grass.jpg")
            # texture_file = chrono.GetChronoDataFile("sensor/textures/mud.png")
            # texture_file = chrono.GetChronoDataFile("sensor/textures/snow.jpg")
            material_list = chrono.CastToChVisualization(patch.GetGroundBody().GetAssets()[0]).material_list

        elif 'scm' in terrain_type:
            self.terrain = veh.SCMDeformableTerrain(self.system)

            # Parameters
            params = SCMParameters()
            if 'hard' in terrain_type:
                params.InitializeParametersAsHard()
            elif 'soft' in terrain_type:
                params.InitializeParametersAsSoft()
            else:
                raise Exception(f'{terrain_type} does not specify SCM parameters (hard or soft)')
            params.SetParameters(self.terrain)

            # Bulldozing
            self.terrain.SetBulldozingFlow(True)
            self.terrain.SetBulldozingParameters(
                55,   # angle of friction for erosion of displaced material at the border of the rut
                1,    # displaced material vs downward pressed material.
                5,    # number of erosion refinements per timestep
                10)   # number of concentric vertex selections subject to erosion


            div_length = self.divs_per_units * self.terrain_length
            div_width = self.divs_per_units * self.terrain_width
            if 'flat' in terrain_type:
                self.terrain.Initialize(0,
                                        self.terrain_length,
                                        self.terrain_width,
                                        int(div_length),
                                        int(div_width))
            elif 'height_map' in terrain_type:
                self.terrain.Initialize(self.bitmap_file,        # heightmap file (.bmp)
                                        "test",                  # mesh name
                                        self.terrain_length,     # sizeX
                                        self.terrain_width,      # sizeY
                                        self.min_terrain_height, # hMin
                                        self.max_terrain_height, # hMax
                                        int(div_length),
                                        int(div_width))

            # Add texture for the terrain
            texture_file = chrono.GetChronoDataFile('sensor/textures/')
            if 'hard' in terrain_type: texture_file += 'mud.png'
            elif 'soft' in terrain_type: texture_file += 'snow.jpg'
            # texture_file = chrono.GetChronoDataFile("sensor/textures/grass_texture.jpg")
            # texture_file = veh.GetDataFile("terrain/textures/grass.jpg")
            # texture_file = chrono.GetChronoDataFile("sensor/textures/mud.png")
            # texture_file = chrono.GetChronoDataFile("sensor/textures/snow.jpg")
            material_list = self.terrain.GetMesh().material_list
        else:
            raise Exception(f'{terrain_type} does not specify terrain type (rigid or scm)')

        vis_mat = chrono.ChVisualMaterial()
        vis_mat.SetSpecularColor(chrono.ChVectorF(0.1, 0.1, 0.1))
        vis_mat.SetFresnelMax(.1)
        vis_mat.SetKdTexture(texture_file)
        material_list.push_back(vis_mat)

        self.initLoc.z = self.terrain.GetHeight(chrono.ChVectorD(x, y, 0)) + 0.5

        self.vehicle = veh.Gator(self.system)
        self.vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
        self.vehicle.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
        self.vehicle.SetChassisFixed(False)
        self.m_inputs = veh.Inputs()
        self.vehicle.SetInitPosition(chrono.ChCoordsysD(self.initLoc, self.initRot))
        self.vehicle.SetTireType(veh.TireModelType_TMEASY)
        if 'scm' in terrain_type:
            self.vehicle.SetTireType(veh.TireModelType_RIGID_MESH)
        self.vehicle.SetTireStepSize(self.timestep)
        self.vehicle.SetAerodynamicDrag(0.5, 5.0, 1.2)
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

        if 'scm' in terrain_type:
            self.terrain.AddMovingPatch(self.vehicle.GetChassisBody(), chrono.ChVectorD(0, 0, 0), 5, 5)

        # Driver
        self.driver = veh.ChDriver(self.vehicle.GetVehicle())

        # create goal
        # pi/4 ang displ
        delta_theta = (random.random()-0.5) * 1.0 * np.pi
        gx, gy = self.terrain_length * 0.5 * np.cos(theta + np.pi + delta_theta), self.terrain_width * 0.5 * np.sin(theta + np.pi + delta_theta)
        self.goal = chrono.ChVectorD(gx, gy, self.terrain.GetHeight(chrono.ChVectorD(gx, gy, 0)) + 1.0)
        # self.goal = chrono.ChVectorD(23.6622, -32.2506, 1)
        gx, gy = -self.initLoc.x, -self.initLoc.y
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
            # self.system.Add(self.goal_sphere)
            pass

        # create obstacles
        # start = t.time()
        self.assets.Clear()
        self.assets.RandomlyPositionAssets(self.system, self.initLoc, self.goal, self.terrain, self.terrain_length, self.terrain_width, should_scale=False)


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
        self.camera = sens.ChCameraSensor(
            self.chassis_body,  # body camera is attached to
            20,  # scanning rate in Hz
            chrono.ChFrameD(chrono.ChVectorD(.65, 0, .75), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))),
            # offset pose
            self.camera_width,  # number of horizontal samples
            self.camera_height,  # number of vertical channels
            chrono.CH_C_PI / 2,  # horizontal field of view
            6
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
            self.assets.TransformAgain()
            # start = t.time()
            # for asset in self.assets.assets:
            #     if len(asset.frames) > 0:
            #         self.manager.AddInstancedStaticSceneMeshes(asset.frames, asset.mesh.shape)
            self.manager.ReconstructScenes()
            # self.manager.AddInstancedStaticSceneMeshes(self.assets.frames, self.assets.shapes)
            # self.manager.Update()
            # print('Reconstruction :: ', t.time() - start)

        self.old_dist = (self.goal - self.initLoc).Length()

        self.assets.write()

        with open('test.txt', 'w') as temp:
            pass
        self.file = open('test.txt', 'a')

        self.step_number = 0
        self.c_f = 0
        self.isdone = False
        self.render_setup = False
        self.dist0 = (self.goal - self.chassis_body.GetPos()).Length()
        if self.play_mode:
            self.render()

        # print(self.get_ob()[1])
        return self.get_ob()

    def step(self, ac):
        self.ac = ac.reshape((-1,))
        # Collect output data from modules (for inter-module communication)

        for i in range(round(1/(self.control_frequency*self.timestep))):
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
            self.driver.Synchronize(time)
            self.vehicle.Synchronize(time, self.m_inputs, self.terrain)
            self.terrain.Synchronize(time)

            # Advance simulation for one timestep for all modules
            self.driver.Advance(self.timestep)
            self.vehicle.Advance(self.timestep)
            self.terrain.Advance(self.timestep)
            self.system.DoStepDynamics(self.timestep)

            self.manager.Update()

            pos = self.chassis_body.GetPos()
            vel = self.chassis_body.GetPos_dt()
            acc = self.chassis_body.GetPos_dtdt()
            t,s,b = self.m_inputs.m_throttle, self.m_inputs.m_steering, self.m_inputs.m_braking

            self.file.write(f'{pos.x},{pos.y},{pos.z},{vel.x},{vel.y},{vel.z},{acc.x},{acc.y},{acc.z},{t},{s},{b}\n')

            self.c_f += self.assets.CalcContactForces(self.chassis_body, self.chassis_collision_box)
            if self.c_f:
                break

            if time + self.timestep > self.step_number:
                print('Time:', int(time + self.timestep))
                self.step_number += 1

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
            # print('NO DATA \n')
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
        # print('x error'+ str(loc_dist_gps[0]-dist_local.x)+ 'y error'+ str(loc_dist_gps[1]-dist_local.y))
        array_data = np.array([loc_dist_gps[0], loc_dist_gps[1], head, targ_head, vel.Length()])
        return (rgb, array_data)

    def calc_rew(self):
        progress_coeff = 20
        # vel_coeff = .01
        time_cost = 0
        progress = self.calc_progress()
        # vel = self.vehicle.GetVehicle().GetVehicleSpeed()
        return progress

    def is_done(self):

        pos = self.chassis_body.GetPos()

        collision = not(self.c_f == 0)
        if self.system.GetChTime() > self.timeend:
            dist = (pos - self.goal).Length()
            print('Timeout!! Distance from goal :: ', dist)
            self.isdone = True
            self.rew -= 400
            failed = 0
        elif abs(pos.x) > self.terrain_length * 1.5 / 2.0 or abs(pos.y) > self.terrain_width * 1.5 / 2 or pos.z < self.min_terrain_height:
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
        #elif abs(self.head_diff)>np.pi/2 :
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
        # print(dist, self.old_dist)
        self.old_dist = dist
        return progress

    def render(self, mode='human'):
        if not (self.play_mode==True):
            raise Exception('Please set play_mode=True to render')

        if not self.render_setup:
            vis = False
            save = True
            birds_eye = False
            third_person = True
            angle = False
            width = 600
            height = 400
            if birds_eye:
                body = chrono.ChBodyAuxRef()
                body.SetBodyFixed(True)
                self.system.AddBody(body)
                vis_camera = sens.ChCameraSensor(
                    body,  # body camera is attached to
                    30,  # scanning rate in Hz
                    chrono.ChFrameD(chrono.ChVectorD(0, 0, 100), chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 1, 0))),
                    # offset pose
                    width,  # number of horizontal samples
                    height,  # number of vertical channels
                    chrono.CH_C_PI / 3,  # horizontal field of view
                    # 0,
                    # 1/20
                )
                vis_camera.SetName("Birds Eye Camera Sensor")
                if vis:
                    vis_camera.PushFilter(sens.ChFilterVisualize(width, height, "Visualization Camera"))
                if save:
                    vis_camera.PushFilter(sens.ChFilterSave("SENSOR_OUTPUT/birds_eye/"))
                self.manager.AddSensor(vis_camera)

            if angle:
                body = chrono.ChBodyAuxRef()
                body.SetBodyFixed(True)
                self.system.AddBody(body)
                vis_camera = sens.ChCameraSensor(
                    body,  # body camera is attached to
                    30,  # scanning rate in Hz
                    chrono.ChFrameD(chrono.ChVectorD(-50, -50, self.initLoc.z+10), chrono.Q_from_Euler123(chrono.ChVectorD(0, chrono.CH_C_PI / 12., chrono.CH_C_PI / 4.))),
                    # offset pose
                    width,  # number of horizontal samples
                    height,  # number of vertical channels
                    chrono.CH_C_PI / 3,  # horizontal field of view
                )
                vis_camera.SetName("Birds Eye Camera Sensor")
                if vis:
                    vis_camera.PushFilter(sens.ChFilterVisualize(width, height, "Visualization Camera"))
                if save:
                    vis_camera.PushFilter(sens.ChFilterSave("SENSOR_OUTPUT/angle/"))
                self.manager.AddSensor(vis_camera)

            if third_person:
                vis_camera = sens.ChCameraSensor(
                    self.chassis_body,  # body camera is attached to
                    30,  # scanning rate in Hz
                    chrono.ChFrameD(chrono.ChVectorD(-6, 0, 2), chrono.Q_from_AngAxis(chrono.CH_C_PI / 20, chrono.ChVectorD(0, 1, 0))),
                    # offset pose
                    width,  # number of horizontal samples
                    height,  # number of vertical channels
                    chrono.CH_C_PI / 3,  # horizontal field of view
                    # 0,
                    # 1/20
                )
                vis_camera.SetName("Follow Camera Sensor")
                if vis:
                    vis_camera.PushFilter(sens.ChFilterVisualize(width, height, "Visualization Camera"))
                if save:
                    vis_camera.PushFilter(sens.ChFilterSave("SENSOR_OUTPUT/third_person/"))
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
        self.file.close()
