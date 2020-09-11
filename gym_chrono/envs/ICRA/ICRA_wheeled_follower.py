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
from gym_chrono.envs.ICRA.assets import *
from gym_chrono.envs.ICRA.parameters import *
from gym_chrono.envs.ICRA.ghost_leader import *
from gym_chrono.envs.ICRA.bezier_path import *

from gym_chrono.envs.ICRA.genetic_algorithm.genetic_algorithm import genetic_algorithm
from gym_chrono.envs.ICRA.genetic_algorithm.common.inputs import Inputs
from gym_chrono.envs.ICRA.genetic_algorithm.common.geometry import Point

# openai-gym imports
import gym
from gym import spaces
from time import time as get_real_time

import signal
import sys

def signal_handler(sig, frame):
    print('Ctrl+C Detected! Exitting...')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class icra_wheeled_follower(ChronoBaseEnv):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}
    def __init__(self):
        ChronoBaseEnv.__init__(self)

        # Set Chrono data directories
        data_dir = os.path.join(os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir)), 'data', '')
        chrono.SetChronoDataPath(data_dir)
        veh.SetDataPath(os.path.join(data_dir, 'vehicle', ''))

        # Define action and observation space
        # They must be gym.spaces objects
        # Example when using discrete actions:
        self.camera_width  = 80
        self.camera_height = 45

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Tuple((
                spaces.Box(low=0, high=255, shape=(self.camera_height, self.camera_width, 3), dtype=np.uint8),  # camera
                spaces.Box(low=-100, high=100, shape=(6,), dtype=np.float)))                                        # goal gps

        self.info =  {"timeout": 10000.0}
        self.timestep = 4e-3

        # -------------------------------
        # Initialize simulation settings
        # -------------------------------

        self.timeend = 30
        self.control_frequency = 5

        self.min_terrain_height = 0     # min terrain height
        self.max_terrain_height = 4 # max terrain height
        self.terrain_length = 180.0 # size in X direction
        self.terrain_width = 180.0  # size in Y direction
        self.divs_per_units = 20 # divisions per unit (SCM only)
        self.bitmap_img_size = tuple(np.array(np.array([self.terrain_length, self.terrain_width]) * self.divs_per_units, dtype=int))

        # Specify the terrain type
        self.terrain_type = ''
        self.terrain_type += 'rigid'
        # self.terrain_type += 'scm_hard'
        # self.terrain_type += 'scm_soft'
        self.terrain_type += '_flat'
        # self.terrain_type += '_height_map'

        self.num_obstacles = 5

        self.rank = -1
        self.num_envs = -1
        self.run_number = 0

        self.save_data = False
        if self.save_data:
            self.folder = 'RL_OUTPUT'

            if 'height_map' in self.terrain_type:
                self.out_dir = f'{self.folder}/{self.terrain_type}{self.max_terrain_height}/{self.num_obstacles}'
            else:
                self.out_dir = f'{self.folder}/{self.terrain_type}/{self.num_obstacles}'

            try:
                os.makedirs(self.out_dir, exist_ok=True)
            except OSError as error:
                print(error)

        self.render_setup = False
        self.play_mode = False

    def get_num(self):
        return self.rank + self.run_number * self.num_envs

    def reset(self):
        self.offset = chrono.ChVectorD(-(10+10*random.random()), pow(-1,random.randint(1,3))*(10+10*random.random()), 0)
        num = self.get_num()
        self.start = get_real_time()
        patch_mat = chrono.ChMaterialSurfaceSMC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
        patch_mat.SetYoungModulus(2e7)

        # Add objects
        n = self.num_obstacles
        b1 = 0 # int(n / 2)
        b2 = 0 # int(n / 2)
        r1 = n
        r2 = n
        r3 = n
        r4 = n
        r5 = n
        t1 = 0 # int(n / 2)
        t2 = 0 # int(n / 2)
        t3 = 0 # int(n / 2)
        c = 0
        if 'soft' in self.terrain_type: b1 = b2 = t1 = t2 = t3 = 0
        self.assets = AssetHandler(patch_mat, b1, b2, r1, r2, r3, r4, r5, t1, t2, t3, c)

        # create leaders
        self.leaders = GhostLeaderHandler(patch_mat, 2, 0.2, "sensor/hmmwv_combined.obj")
        self.target = chrono.ChVectorD()
        # Create systems
        #TODO: NSC sys?
        #TODO: tweak solver & TS for speed
        self.system = chrono.ChSystemSMC()
        self.system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
        self.system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
        self.system.SetSolverMaxIterations(50)
        self.system.SetMaxPenetrationRecoverySpeed(4.0)

        theta = random.random()*2*np.pi
        x,y = self.terrain_length*0.45*np.cos(theta) , self.terrain_width*0.45*np.sin(theta)
        ang = np.pi + theta
        #x,y = pow(-1, np.random.randint(1,))*0.9*self.terrain_length ,
        self.initLoc = chrono.ChVectorD(x, y, 0)
        self.initRot = chrono.Q_from_AngZ(np.pi / 4)

        # Create terrain
        if 'height_map' in self.terrain_type:
            shape = (252, 252)
            if self.save_data:
                self.bitmap_file = os.path.join(self.out_dir, f'height_map{num}.png')
            else:
                self.bitmap_file = f'height_map{num}.png'
            generate_random_bitmap(shape=shape, resolutions=[(2, 2)], mappings=[(-1.5,1.5)], img_size=(100,100), file_name=self.bitmap_file, initPos=self.initLoc)

        if 'rigid' in self.terrain_type:
            self.terrain = veh.RigidTerrain(self.system)

            if 'flat' in self.terrain_type:
                patch = self.terrain.AddPatch(patch_mat,
                                         chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1),
                                         self.terrain_length, self.terrain_width)
            elif 'height_map' in self.terrain_type:
                patch = self.terrain.AddPatch(patch_mat,
                                            chrono.CSYSNORM,       # position
                                            self.bitmap_file,        # heightmap file (.bmp)
                                            "test",                  # mesh name
                                            self.terrain_length,     # sizeX
                                            self.terrain_width,      # sizeY
                                            self.min_terrain_height, # hMin
                                            self.max_terrain_height) # hMax

            self.terrain.Initialize()
            #TODO: randomize terrain patch
            texture_file = chrono.GetChronoDataFile("sensor/textures/grass_texture.jpg")
            material_list = chrono.CastToChVisualization(patch.GetGroundBody().GetAssets()[0]).material_list

        elif 'scm' in self.terrain_type:
            self.terrain = veh.SCMDeformableTerrain(self.system)

            # Parameters
            params = SCMParameters()
            if 'hard' in self.terrain_type:
                params.InitializeParametersAsHard()
            elif 'soft' in self.terrain_type:
                params.InitializeParametersAsSoft()
            else:
                raise Exception(f'{self.terrain_type} does not specify SCM parameters (hard or soft)')
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
            if 'flat' in self.terrain_type:
                self.terrain.Initialize(0,
                                        self.terrain_length,
                                        self.terrain_width,
                                        int(div_length),
                                        int(div_width))
            elif 'height_map' in self.terrain_type:
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
            if 'hard' in self.terrain_type: texture_file += 'mud.png'
            elif 'soft' in self.terrain_type: texture_file += 'snow.jpg'
            material_list = self.terrain.GetMesh().material_list
        else:
            raise Exception(f'{self.terrain_type} does not specify terrain type (rigid or scm)')

        vis_mat = chrono.ChVisualMaterial()
        vis_mat.SetSpecularColor(chrono.ChVectorF(0.1, 0.1, 0.1))
        vis_mat.SetFresnelMax(.1)
        vis_mat.SetKdTexture(texture_file)
        material_list.push_back(vis_mat)

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

        # create obstacles
        self.assets.RandomlyPositionAssets(self.system, self.initLoc, self.goal, self.terrain, self.terrain_length, self.terrain_width, should_scale=True)

        # create path
        start = [self.initLoc.x, self.initLoc.y]
        goal = [self.goal.x, self.goal.y]
            

        inputs = Inputs('genetic')

        inputs.grid_minimum = Point(-self.terrain_width / 2, -self.terrain_length / 2)
        inputs.grid_maximum = Point(self.terrain_width / 2, self.terrain_length / 2)
        inputs.curve_samples = 30
        inputs.population_count = 50
        inputs.start = Point(self.initLoc.x,self.initLoc.y)
        inputs.goal = Point(self.goal.x,self.goal.y)
        inputs.obstacles = self.assets.assets
        inputs.plot = False
        
        optimal_path = genetic_algorithm(inputs)

        points = chrono.vector_ChVectorD()
        for p in optimal_path.points:
            points.push_back(chrono.ChVectorD(p.x, p.y, 0.2))

        self.path = BezierPath(points)

        _,self.initRot = self.path.GetPose(0)
        self.initLoc.z = self.terrain.GetHeight(chrono.ChVectorD(x, y, 0)) + 0.75

        self.vehicle = veh.HMMWV_Reduced(self.system)
        self.vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
        self.vehicle.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
        self.vehicle.SetChassisFixed(False)
        self.vehicle.SetInitPosition(chrono.ChCoordsysD(self.initLoc, self.initRot))
        self.vehicle.SetTireType(veh.TireModelType_TMEASY)
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

        self.m_inputs = veh.Inputs()

        self.chassis_body.GetCollisionModel().SetFamily(2)
        self.chassis_body.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(0)
        self.chassis_body.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(1)

        if 'scm' in self.terrain_type:
            self.terrain.AddMovingPatch(self.vehicle.GetChassisBody(), chrono.ChVectorD(0, 0, 0), 5, 5)

        # Driver
        self.driver = veh.ChDriver(self.vehicle.GetVehicle())

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
            pass


        self.leaders.Initialize(self.system, self.path)
        for leader in self.leaders:
            self.assets.AddAsset(leader)
        
        self.lead_pos = (self.chassis_body.GetPos() - self.leaders[0].body.GetPos()).Length()
        self.leader_totalsteps = self.timeend / self.timestep
        self.total_steps = 0
        self.opt_dist = 12
        self.dist_rad = 4

        # for i in np.arange(0,1,1/1000.):
        #     p = self.path.eval(i)
        #     sphere = chrono.ChBodyEasySphere(.25, 1000, True, False)
        #     sphere.SetBodyFixed(True)
        #     sphere.SetPos(p)
        #     sphere.AddAsset(chrono.ChColorAsset(1,0,0))
        #     self.system.Add(sphere)

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

        # -----------------------------------------------------
        # Create a self.camera and add it to the sensor manager
        # -----------------------------------------------------
        self.camera = sens.ChCameraSensor(
            self.chassis_body,  # body camera is attached to
            20,  # scanning rate in Hz
            chrono.ChFrameD(chrono.ChVectorD(1.5, 0, .875)),
            # offset pose
            self.camera_width,  # number of horizontal samples
            self.camera_height,  # number of vertical channels
            chrono.CH_C_PI / 2,  # horizontal field of view
            6
        )
        self.camera.SetName("Camera Sensor")
        self.camera.PushFilter(sens.ChFilterRGBA8Access())
        if self.play_mode:
           self.camera.PushFilter(sens.ChFilterVisualize(self.camera_width, self.camera_height, "RGB Camera"))
        self.manager.AddSensor(self.camera)

        # ----------------------------------
        # Create a gps attached to the agent
        # ----------------------------------
        gps_noise_none = sens.ChGPSNoiseNone()
        self.agent_gps = sens.ChGPSSensor(
            self.chassis_body,
            15,
            chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))),
            self.origin,
            gps_noise_none
        )
        self.agent_gps.SetName("GPS Sensor")
        self.agent_gps.PushFilter(sens.ChFilterGPSAccess())
        self.manager.AddSensor(self.agent_gps)

        # -----------------------------------
        # Create a gps attached to the leader
        # -----------------------------------
        gps_noise_none = sens.ChGPSNoiseNone()
        self.leader_gps = sens.ChGPSSensor(
            self.leaders[0].body,
            15,
            chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))),
            self.origin,
            gps_noise_none
        )
        self.leader_gps.SetName("GPS Sensor")
        self.leader_gps.PushFilter(sens.ChFilterGPSAccess())
        self.manager.AddSensor(self.leader_gps)

        self.assets.ResetAssets()
        self.manager.ReconstructScenes()

        self.old_dist = (self.goal - self.initLoc).Length()

        if self.save_data:
            sim_data_filename = os.path.join(self.out_dir, f'sim_data{num}.txt')
            obs_filename = os.path.join(self.out_dir, f'obs{num}.txt')
            self.hit_obs_filename = os.path.join(self.out_dir, f'hit_obs{num}.txt')
            self.results_filename = os.path.join(self.out_dir, f'results{num}.txt')

            self.sim_data_file = open(sim_data_filename, 'w')
            temp = open(self.hit_obs_filename, 'w'); temp.close()

            self.assets.Write(filename=obs_filename)

            # self.driver_file = open(os.path.join(self.out_dir, f'sim_data{num}.txt'))

        self.run_number += 1

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

            # line = self.driver_file.readline()
            # (t,s,b) = line.split(',')[11:14]
            # t,s,b=0,0,[1]
            # self.m_inputs.m_throttle = float(t)
            # self.m_inputs.m_steering = float(s)
            # self.m_inputs.m_braking = float(b[0])

            self.driver.Synchronize(time)
            self.vehicle.Synchronize(time, self.m_inputs, self.terrain)
            self.terrain.Synchronize(time)

            # Advance simulation for one timestep for all modules
            self.driver.Advance(self.timestep)
            self.vehicle.Advance(self.timestep)
            self.terrain.Advance(self.timestep)
            self.system.DoStepDynamics(self.timestep)

            self.path.Advance((1 / self.leader_totalsteps)*((2*self.total_steps)/self.leader_totalsteps))
            self.leaders.Update()
            self.manager.Update()
            self.target.Set(self.leaders[0].body.GetPos() + self.leaders[0].body.GetRot().Rotate(self.offset)  )
            self.goal_sphere.SetPos(self.target + chrono.ChVectorD(0,0,10))
            self.total_steps+=1

            if self.save_data:
                pos = self.chassis_body.GetPos()
                vel = self.chassis_body.GetPos_dt()
                acc = self.chassis_body.GetPos_dtdt()
                head = self.vehicle.GetVehicle().GetVehicleRot().Q_to_Euler123().z
                t,s,b = self.m_inputs.m_throttle, self.m_inputs.m_steering, self.m_inputs.m_braking

                self.sim_data_file.write(f'{time},{pos.x},{pos.y},{pos.z},{vel.x},{vel.y},{vel.z},{acc.x},{acc.y},{acc.z},{head},{t},{s},{b}\n')

            contacted_assets = self.assets.GetContactedAssets()
            if len(contacted_assets):
                print('Contact!')
                if self.save_data:
                    self.assets.Write(self.hit_obs_filename, contacted_assets)
                self.c_f = 1
                break

            if self.play_mode and time + self.timestep > self.step_number:
                print('Time:', int(time + self.timestep))
                self.step_number += 1
            
        self.obs = self.get_ob()
        self.rew = self.calc_rew()
        self.is_done()

        if self.isdone:
            print(f'Total time: {get_real_time() - self.start}')
        return self.obs, self.rew, self.isdone, self.info

    def get_ob(self):
        camera_buffer_RGBA8 = self.camera.GetMostRecentRGBA8Buffer()
        if camera_buffer_RGBA8.HasData():
            rgb = camera_buffer_RGBA8.GetRGBA8Data()[:, :, 0:3]
        else:
            rgb = np.zeros((self.camera_height, self.camera_width, 3))

        gps_buffer = self.agent_gps.GetMostRecentGPSBuffer()
        if gps_buffer.HasData():
            cur_gps_data = gps_buffer.GetGPSData()
            cur_gps_data = chrono.ChVectorD(cur_gps_data[1], cur_gps_data[0], cur_gps_data[2])
        else:
            cur_gps_data = chrono.ChVectorD(self.origin)
        
        gps_buffer = self.leader_gps.GetMostRecentGPSBuffer()
        if gps_buffer.HasData():
            lead_gps_data = gps_buffer.GetGPSData()
            lead_gps_data = chrono.ChVectorD(lead_gps_data[1], lead_gps_data[0], lead_gps_data[2])
        else:            
            lead_gps_data = chrono.ChVectorD(self.origin)


        gps_data = [(self.goal - self.chassis_body.GetPos()).x, (self.goal - self.chassis_body.GetPos()).y]
        speed = self.chassis_body.GetPos_dt().Length()
        head = self.vehicle.GetVehicle().GetVehicleRot().Q_to_Euler123().z
        lead_pos = self.leaders[0].body.GetPos().Length()
        lead_speed = (self.lead_pos - lead_pos)*self.control_frequency
        lead_head = self.leaders[0].body.GetRot().Q_to_Euler123().z
        #dist = self.target - self.chassis_body.GetPos()
        #dist_local = self.chassis_body.GetRot().RotateBack(dist)

        leadCart = chrono.ChVectorD(lead_gps_data)
        sens.GPS2Cartesian(leadCart, self.origin)
        sens.GPS2Cartesian(cur_gps_data, self.origin)
        # TODO: rotate the offset using the angle for better represenation of the real scenario
        gps_dist = leadCart + self.leaders[0].body.GetRot().Rotate(self.offset) - cur_gps_data
        loc_dist_gps = [gps_dist.x * np.cos(head) + gps_dist.y * np.sin(head),
                        -gps_dist.x * np.sin(head) + gps_dist.y * np.cos(head)]
        targ_head = np.arctan2(loc_dist_gps[1], loc_dist_gps[0])
        #Check quantities
        """
        dist_local_ref = self.chassis_body.GetRot().RotateBack(dist)
        targ_head_ref = np.arctan2(dist_local_ref.y, dist_local_ref.x)
        print(str(dist_local_ref.x-loc_dist_gps[0]) +str(dist_local_ref.y-loc_dist_gps[1]))
        """
        return (rgb, np.concatenate([loc_dist_gps, [head], [targ_head], [speed], [lead_speed]]))


    def calc_rew(self):
        proximity = 20
        eps = 0.05 # avoid division by 0
        self.dist = (self.chassis_body.GetPos() - self.target).Length()
        rew = self.calc_progress() + proximity * (1/(self.dist+eps))
        # vel = self.vehicle.GetVehicle().GetVehicleSpeed()
        return rew

    def calc_progress(self):
        dist = (self.chassis_body.GetPos() - self.target).Length()
        progress = self.old_dist - dist
        # print(dist, self.old_dist)
        self.old_dist = dist
        return progress

    def is_done(self):

        def save_results(result, reason):
            if self.save_data:
                with open(self.results_filename, 'w') as file:
                    file.write(f'{result},{reason}')

        pos = self.chassis_body.GetPos()

        collision = not(self.c_f == 0)
        # The episodes end with the agent in the right place
        if self.system.GetChTime() > self.timeend and self.dist < 3:
            self.rew += 2500
            print('Success!!')
            # self.successes += 1
            self.isdone = True
            failed = 3
            save_results('Success', '')

        elif abs(pos.x) > self.terrain_length / 2.0 or abs(pos.y) > self.terrain_width / 2 or pos.z < self.min_terrain_height:
            dist = (self.chassis_body.GetPos() - self.goal).Length()
            print('Fell off terrain!! Distance from goal :: ', dist)
            # self.rew -= 250
            self.isdone = True
            failed = 1
            save_results('Failed','Off')
        elif collision:
            self.rew -= 250
            print('Hit object')
            self.isdone = True
            failed = 2
            save_results('Failed','Hit')
        elif self.dist > 80:
            print('Too far from leader!')
            self.isdone = True
            failed = 4
            save_results('Failed','Leader')
        elif self.system.GetChTime() > self.timeend:
            dist = (pos - self.goal).Length()
            print('Timeout!! Distance from goal :: ', dist)
            self.isdone = True
            failed = 0
            save_results('Failed', 'Timeout')

    def render(self, mode='human'):
        if not (self.play_mode==True):
            raise Exception('Please set play_mode=True to render')

        if not self.render_setup:
            # res = (1920,1280)
            res = (800,600)
            vis = 1
            save = 0
            birds_eye = 1
            third_person = 0
            angle = 0
            close = 0
            width = res[0]
            height = res[1]
            if birds_eye:
                body = chrono.ChBodyAuxRef()
                body.SetBodyFixed(True)
                self.system.AddBody(body)
                q = chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 1, 0))
                q = chrono.Q_from_AngZ(chrono.CH_C_PI / 2) * q
                vis_camera = sens.ChCameraSensor(
                    body,  # body camera is attached to
                    30,  # scanning rate in Hz
                    chrono.ChFrameD(chrono.ChVectorD(0, 0, 150), q),
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

            if close:
                vis_camera = sens.ChCameraSensor(
                    self.chassis_body,  # body camera is attached to
                    30,  # scanning rate in Hz
                    chrono.ChFrameD(chrono.ChVectorD(-2, -10, 2), chrono.Q_from_Euler123(chrono.ChVectorD(0, chrono.CH_C_PI / 12., chrono.CH_C_PI / 2.))),
                    # offset pose
                    width,  # number of horizontal samples
                    height,  # number of vertical channels
                    chrono.CH_C_PI / 3,  # horizontal field of view
                )
                vis_camera.SetName("Birds Eye Camera Sensor")
                if vis:
                    vis_camera.PushFilter(sens.ChFilterVisualize(width, height, "Visualization Camera"))
                if save:
                    vis_camera.PushFilter(sens.ChFilterSave("SENSOR_OUTPUT/close/"))
                self.manager.AddSensor(vis_camera)

            if third_person:
                vis_camera = sens.ChCameraSensor(
                    self.chassis_body,  # body camera is attached to
                    30,  # scanning rate in Hz
                    chrono.ChFrameD(chrono.ChVectorD(-11, 0, 4), chrono.Q_from_AngAxis(chrono.CH_C_PI / 17, chrono.ChVectorD(0, 1, 0))),
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
        del self.manager
        try:
            self.file.close()
            # self.driver_file.close()
        except:
            return
