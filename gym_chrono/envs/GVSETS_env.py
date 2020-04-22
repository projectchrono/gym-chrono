# TODO list:
# domain randomization (speed  and leader interval correlation)

# PyChrono imports
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

# Default lib imports
import numpy as np
import math as m
import pathlib
from random import randint

# Custom imports
from gym_chrono.envs.ChronoBase import ChronoBaseEnv
from control_utilities.chrono_utilities import setDataDirectory
from control_utilities.driver import Driver
from control_utilities.obstacle import getObstacleBoundaryDim

# openai-gym imports
from gym import spaces


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
def areColliding(body1, body2, box1, box2):
    pos1, rot1, pos2,rot2 = body1.GetPos(), body1.GetRot(), body2.GetPos(), body2.GetRot()
    for i in range(4):
        s = i%2 , m.floor(i/2)
        a, b = box2[0]*s[0]/2, box2[1]*s[1]/2
        p = pos2 + rot2.Rotate(chrono.ChVectorD(a,b,pos2.z))
        d = rot1.RotateBack(p-pos1)
        if abs(d.x)<box1[0]/2 and abs(d.y)<box1[1]/2:
            return True
    return False

class ghostLeaders(object):
    def __init__(self, numlead, interval = 0.05):
        self.interval = interval
        self.numlead = numlead
        self.leaders = []
        self.vis_mesh = chrono.ChTriangleMeshConnected()
        self.vis_mesh.LoadWavefrontMesh(str(pathlib.Path(__file__).parent.absolute()) +
                                        "/data/sensor/hmmwv_combined.obj", True, True)
        self.box = getObstacleBoundaryDim(self.vis_mesh)
        self.trimesh_shape = chrono.ChTriangleMeshShape()
        # trimesh_shape.SetMesh(meshes)
        self.trimesh_shape.SetMesh(self.vis_mesh)
        self.trimesh_shape.SetName("mesh_name")
        self.trimesh_shape.SetStatic(True)

    def addLeaders(self, system, path):
        self.leaders = []
        self.path = path
        for i in range(self.numlead):
            leader = chrono.ChBody()
            leader.AddAsset(self.trimesh_shape)
            system.Add(leader)
            self.leaders.append(leader)
        self.Update()

    def getBBox(self):
        return self.box

    def __getitem__(self, item):
        return self.leaders[item]
    def Update(self):
        t = self.path.current_t
        for i, leader in enumerate(self.leaders):
            leaderPos, leaderRot = self.path.getPosRot(t + i*self.interval)
            leader.SetPos(leaderPos)
            leader.SetRot(leaderRot)

class BezierPath(chrono.ChBezierCurve):
    def __init__(self, beginPos, endPos, z):
        # making 4 turns to get to the end point
        deltaX = (endPos[0] - beginPos[0])/3
        deltaY = (endPos[1] - beginPos[1])/2
        points = chrono.vector_ChVectorD()
        for i in range(6):
            point = chrono.ChVectorD(beginPos[0] + deltaX*m.floor((i+1)/2) , beginPos[1] + deltaY*m.floor(i/2), z)
            points.append(point)
        super(BezierPath, self).__init__(points)
        self.current_t = 0

    # Update the progress on the path of the leader
    def Advance(self, delta_t):
        self.current_t += delta_t

    def getPoints(self):
        points = []
        for i in range(self.getNumPoints()):
            points.append(self.getPoint(i))
        return points

    # Param-only derivative
    def par_evalD(self, t):
        par = np.clip(t, 0.0, 1.0)
        numIntervals = self.getNumPoints() - 1
        epar = par * numIntervals
        i = m.floor(par * numIntervals)
        i = np.clip(i, 0, numIntervals - 1)
        return self.evalD(int(i), epar - i)

    # Current positon and rotation of the leader vehicle chassis
    def getPosRot(self, t):
        pos = self.eval(t)
        posD = self.par_evalD(t)
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
        #self.camera_width = 210
        self.camera_width = 80
        #self.camera_height = 160
        self.camera_height = 45
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Tuple((spaces.Box(low=0, high=255, shape=(self.camera_height, self.camera_width, 3), dtype=np.uint8),  # camera
                                                spaces.Box(low=-100, high=100, shape=(2,), dtype=np.float)))
        self.info = {"timeout": 10000.0}
        self.timestep = 5e-3
        # ---------------------------------------------------------------------
        #
        #  Create the simulation system and add items
        #
        self.timeend = 40
        self.opt_dist = 8
        self.dist_rad = 4
        self.control_frequency = 5
        # time needed by the leader to get to the end of the path
        self.leader_totalsteps = self.timeend / self.timestep
        self.terrain_model = veh.RigidTerrain.PatchType_BOX
        self.terrainHeight = 0  # terrain height (FLAT terrain only)
        self.terrainLength = 200.0  # size in X direction
        self.terrainWidth = 200.0  # size in Y direction
        self.obst_paths = ['sensor/offroad/rock1.obj', 'sensor/offroad/rock3.obj', 'sensor/offroad/rock4.obj',
                      'sensor/offroad/rock5.obj', 'sensor/offroad/tree1.obj', 'sensor/offroad/bush.obj']
        self.vis_meshes = [chrono.ChTriangleMeshConnected() for i in range(len(self.obst_paths))]
        for path, mesh in zip(self.obst_paths, self.vis_meshes):  mesh.LoadWavefrontMesh(chrono.GetChronoDataFile(path), True, True)
        self.obst_bound = [ getObstacleBoundaryDim(mesh) for mesh in self.vis_meshes]
        self.trimeshes = []
        for vis_mesh in self.vis_meshes:
            trimesh_shape = chrono.ChTriangleMeshShape()
            trimesh_shape.SetMesh(vis_mesh)
            trimesh_shape.SetName("mesh_name")
            trimesh_shape.SetStatic(True)
            self.trimeshes.append(trimesh_shape)
        self.leaders = ghostLeaders(3)
        self.origin = chrono.ChVectorD(-89.400, 43.070, 260.0)  # Origin being somewhere in Madison WI
        # Set the time response for steering and throttle inputs.
        # NOTE: this is not exact, since we do not render quite at the specified FPS.
        steering_time = 1.0
        # time to go from 0 to +1 (or from 0 to -1)
        throttle_time = .5
        # time to go from 0 to +1
        braking_time = 0.3
        # time to go from 0 to +1
        self.SteeringDelta = (self.timestep / steering_time)
        self.ThrottleDelta = (self.timestep / throttle_time)
        self.BrakingDelta = (self.timestep / braking_time)
        self.render_setup = False
        self.play_mode = False
        self.step_number = 0

    def placeObstacle(self, numob):
        for i in range(numob):
            side = randint(1,2)
            path = randint(0,len(self.obst_paths)-1)
            obst = chrono.ChBody()
            #vis_mesh = self.vis_meshes[path]
            obst.AddAsset(self.trimeshes[path])
            x, y, z = self.obst_bound[path]
            obst.GetCollisionModel().ClearModel()
            obst.GetCollisionModel().AddBox(x / 2, y / 2, z / 2)  # must set half sizes
            obst.GetCollisionModel().BuildModel()
            obst.SetCollide(True)
            p0, q = self.path.getPosRot( (i+1)/(numob+1) )
            dist = np.max([x,y]) + self.leader_box[1]
            pos = p0 + q.RotateBack(chrono.VECT_Y) * (dist* pow(-1,side)) -chrono.ChVectorD(0,0,p0.z)
            obst.SetPos(pos)
            obst.SetBodyFixed(True)
            self.obstacles.append(obst)
            self.system.Add(obst)

    def reset(self):
        flip = pow(-1, randint(0,1))
        leader_initloc = [-90, -40*flip]
        leader_endloc = [90, 40*flip]
        self.initLoc = chrono.ChVectorD(leader_initloc[0] - 5, leader_initloc[1] - 5, 1)
        self.initRot = chrono.ChQuaternionD(1, 0, 0, 0)
        self.path = BezierPath(leader_initloc, leader_endloc, 0.5)
        self.vehicle = veh.HMMWV_Reduced()
        self.vehicle.SetContactMethod(chrono.ChMaterialSurface.NSC)
        self.surf_material = chrono.ChMaterialSurfaceNSC()
        self.vehicle.SetChassisCollisionType(veh.ChassisCollisionType_PRIMITIVES)

        self.vehicle.SetChassisFixed(False)
        self.vehicle.SetInitPosition(chrono.ChCoordsysD(self.initLoc, self.initRot))
        self.vehicle.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
        self.vehicle.SetDriveType(veh.DrivelineType_AWD)
        # self.vehicle.SetSteeringType(veh.SteeringType_PITMAN_ARM)
        self.vehicle.SetTireType(veh.TireModelType_TMEASY)
        self.vehicle.SetTireStepSize(self.timestep)
        self.vehicle.Initialize()
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

        self.terrain = veh.RigidTerrain(self.system)
        patch = self.terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, self.terrainHeight - 5), chrono.QUNIT),
                                      chrono.ChVectorD(self.terrainLength, self.terrainWidth, 10))
        patch.SetContactFrictionCoefficient(0.9)
        patch.SetContactRestitutionCoefficient(0.01)
        patch.SetContactMaterialProperties(2e7, 0.3)
        patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
        patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
        self.terrain.Initialize()
        self.groundBody = patch.GetGroundBody()
        ground_asset = self.groundBody.GetAssets()[0]
        visual_asset = chrono.CastToChVisualization(ground_asset)
        vis_mat = chrono.ChVisualMaterial()
        vis_mat.SetKdTexture(veh.GetDataFile("terrain/textures/grass.jpg"))
        visual_asset.material_list.append(vis_mat)
        self.leaders.addLeaders(self.system, self.path)
        self.leader_box = self.leaders.getBBox()
        # Add obstacles:
        self.obstacles = []
        self.placeObstacle(8)

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
        self.camera.FilterList().append(sens.ChFilterRGBA8Access())
        # -----------------------------------------------------
        # Create a self.gps and add it to the sensor manager
        # -----------------------------------------------------
        gps_noise_none = sens.ChGPSNoiseNone()
        self.AgentGPS = sens.ChGPSSensor(
            self.chassis_body,
            100,
            chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))),
            self.origin,
            gps_noise_none
        )
        self.AgentGPS.SetName("AgentGPS Sensor")
        self.AgentGPS.FilterList().append(sens.ChFilterGPSAccess())
        self.manager.AddSensor(self.AgentGPS)
        ### Target GPS
        self.TargetGPS = sens.ChGPSSensor(
            self.leaders[0],
            100,
            chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))),
            self.origin,
            gps_noise_none
        )
        self.TargetGPS.SetName("TargetGPS Sensor")
        self.TargetGPS.FilterList().append(sens.ChFilterGPSAccess())
        self.manager.AddSensor(self.TargetGPS)


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
            steering = np.clip(self.ac[0,], self.driver.GetSteering() - self.SteeringDelta,
                               self.driver.GetSteering() + self.SteeringDelta)
            if self.ac[1,] > 0:
                throttle = np.clip(abs(self.ac[1,]), self.driver.GetThrottle() - self.ThrottleDelta,
                                   self.driver.GetThrottle() + self.ThrottleDelta)
                braking = np.clip(0, self.driver.GetBraking() - self.BrakingDelta,
                                  self.driver.GetBraking() + self.BrakingDelta)
            else:
                braking = np.clip(abs(self.ac[1,]), self.driver.GetBraking() - self.BrakingDelta,
                                  self.driver.GetBraking() + self.BrakingDelta)
                throttle = np.clip(0, self.driver.GetThrottle() - self.ThrottleDelta,
                                   self.driver.GetThrottle() + self.ThrottleDelta)
            self.driver.SetSteering(steering)
            self.driver.SetThrottle(throttle)
            self.driver.SetBraking(braking)
            # Advance simulation for one timestep for all modules
            self.driver.Advance(self.timestep)
            self.vehicle.Advance(self.timestep)
            self.terrain.Advance(self.timestep)
            #self.system.DoStepDynamics(self.timestep)
            self.path.Advance(1 / self.leader_totalsteps)
            self.leaders.Update()
            self.manager.Update()
            self.step_number += 1
            for obs in self.obstacles:
                self.c_f += obs.GetContactForce().Length()
        self.leaderColl = any(areColliding(self.chassis_body, leader, self.leader_box, self.leader_box) for leader in self.leaders)
        self.rew = self.calc_rew()
        self.obs = self.get_ob()
        self.is_done()
        return self.obs, self.rew, self.isdone, self.info

    def get_ob(self):
        camera_data_RGBA8 = self.camera.GetMostRecentRGBA8Buffer()
        if camera_data_RGBA8.HasData():
            rgb = camera_data_RGBA8.GetRGBA8Data()[:, :, 0:3]
        else:
            rgb = np.zeros((self.camera_height, self.camera_width, 3))

        agent_gps_buffer = self.AgentGPS.GetMostRecentGPSBuffer()
        if agent_gps_buffer.HasData():
            agent_gps_data = agent_gps_buffer.GetGPSData()[0:2]
        else:
            agent_gps_data = np.array([self.origin.x, self.origin.y])#, self.origin.z])
        target_gps_buffer = self.TargetGPS.GetMostRecentGPSBuffer()
        if target_gps_buffer.HasData():
            targ_gps_data = target_gps_buffer.GetGPSData()[0:2]
        else:
            targ_gps_data = np.array([self.origin.x, self.origin.y])#, self.origin.z])
        gps = (targ_gps_data - agent_gps_data)*100000
        return rgb, gps

    def calc_rew(self):
        dist_coeff = 20
        eps = 5e-2
        # the target is BEHIND the last leader
        target = self.leaders[0].GetPos() + self.leaders[0].GetRot().Rotate(chrono.ChVectorD(-self.opt_dist, 0, 0))
        pos = self.chassis_body.GetPos()
        self.dist = np.linalg.norm( [target.x - pos.x, target.y - pos.y])
        # extend optimal area by the radius
        rew = dist_coeff /( max(self.dist-self.dist_rad,0) + eps)
        """
        if self.dist > self.opt_dist:
            rew = -0.15*pow(self.dist-10,2)
        else:
            rew =  -pow(self.dist-10,2)
        """
        return rew

    def is_done(self):
        collision = not (self.c_f == 0)
        if self.system.GetChTime() > self.timeend:
            print("Over self.timeend")
            #self.rew += 2000
            self.isdone = True

        elif collision or self.dist>50 or self.leaderColl:
            #self.rew += - 2000
            self.isdone = True

    def render(self, mode='human'):
        if not (self.play_mode == True):
            raise Exception('Please set play_mode=True to render')

        if not self.render_setup:
            if True:
                vis_camera = sens.ChCameraSensor(
                    self.groundBody,  # body camera is attached to
                    30,  # scanning rate in Hz
                    chrono.ChFrameD(chrono.ChVectorD(0, 0, 285), chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 1, 0))),
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
                    self.leaders[0],  # body camera is attached to
                    30,  # scanning rate in Hz
                    chrono.ChFrameD(chrono.ChVectorD(-6, 0, 1.5),
                                    chrono.Q_from_AngAxis(chrono.CH_C_PI / 10, chrono.ChVectorD(0, 1, 0))),
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

    def close(self):
        del self

    def ScreenCapture(self, interval):
        raise NotImplementedError

    def __del__(self):
        del self.manager
