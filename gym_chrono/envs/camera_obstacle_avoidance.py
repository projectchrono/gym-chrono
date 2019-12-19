import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr
from driver import Driver
import numpy as np
import math
import os
from gym_chrono.envs.ChronoBase import  ChronoBaseEnv

# openai-gym imports
import gym
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
"""
try:
    chrono.SetChronoDataPath(os.environ['CHRONO_DATA_DIR'])
    veh.SetDataPath(os.path.join(os.environ['CHRONO_DATA_DIR'], 'vehicle', ''))
except:
    raise Exception('Cannot find CHRONO_DATA_DIR environmental variable. Explanation located in chrono_sim.py file')
"""
def checkFile(file):
    if not os.path.exists(file):
        raise Exception('Cannot find {}. Explanation located in chrono_sim.py file'.format(file))

chrono.SetChronoDataPath('/home/simonebenatti/codes/Chrono/chrono/data/')
veh.SetDataPath('/home/simonebenatti/codes/Chrono/chrono/data/vehicle/')
"""
def GetInitPose(p1, p2, z=0.55, reversed=0):
    initLoc = chrono.ChVectorD(p1[0], p1[1], z)

    vec = chrono.ChVectorD(p2[0], p2[1], z) - chrono.ChVectorD(p1[0], p1[1], z)
    theta = math.atan2((vec%chrono.ChVectorD(1,0,0)).Length(),vec^chrono.ChVectorD(1,0,0))
    if reversed:
        theta *= -1
    initRot = chrono.ChQuaternionD()
    initRot.Q_from_AngZ(theta)

    return initLoc, initRot
"""
class self.camera_obstacle_avoidance(ChronoBaseEnv):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}
    def __init__(self):
        super(self.camera_obstacle_avoidance, self).__init__()

        # Define action and observation space
        # They must be gym.spaces objects
        # Example when using discrete actions:
        self.action_space = spaces.Discrete(3)
        self.observation_space = spaces.Box(9,)

        self.info =  {"timeout": 10000.0}
        self.timestep = 3e-3
        # ---------------------------------------------------------------------
        #
        #  Create the simulation system and add items
        #
        self.Xtarg = 100.0
        self.Ytarg = 0.0
        self.timeend = 15
        self.control_frequency = 50
        # JSON file for vehicle model
        self.vehicle_file = veh.GetDataPath() + os.path.join('hmmwv', 'vehicle', 'HMMWV_Vehicle.json')
        checkFile(self.vehicle_file)

        # JSON file for powertrain (simple)
        self.simplepowertrain_file = veh.GetDataPath() + os.path.join('generic', 'powertrain', 'SimplePowertrain.json')
        checkFile(self.simplepowertrain_file)

        # JSON files tire models (rigid)
        self.rigidtire_file = veh.GetDataPath() + os.path.join('hmmwv', 'tire', 'HMMWV_RigidTire.json')
        checkFile(self.rigidtire_file)

        self.terrain_model = veh.RigidTerrain.BOX
        self.terrainHeight = 0  # terrain height (FLAT terrain only)
        self.terrainLength = 250.0  # size in X direction
        self.terrainWidth = 15.0  # size in Y direction

        self.render_setup = False

    def reset(self):
        self.vehicle = veh.WheeledVehicle(self.vehicle_file, chrono.ChMaterialSurface.NSC)
        self.vehicle.Initialize(chrono.ChCoordsysD(self.initLoc, self.initRot))
        self.vehicle.SetStepsize(self.timestep)
        self.vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
        self.vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
        self.vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
        self.vehicle.SetWheelVisualizationType(veh.VisualizationType_NONE)
        self.chassis_body = self.vehicle.GetChassisBody()

        # Rigid terrain
        self.system = self.vehicle.GetSystem()
        terrain = veh.RigidTerrain(self.system)
        patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, self.terrainHeight - 5), chrono.QUNIT),
                                 chrono.ChVectorD(self.terrainLength, self.terrainWidth, 10))
        patch.SetContactFrictionCoefficient(0.9)
        patch.SetContactRestitutionCoefficient(0.01)
        patch.SetContactMaterialProperties(2e7, 0.3)
        patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
        patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
        terrain.Initialize()

        ground_body = patch.GetGroundBody()
        ground_asset = ground_body.GetAssets()[0]
        visual_asset = chrono.CastToChVisualization(ground_asset)
        vis_mat = chrono.ChVisualMaterial()
        vis_mat.SetKdTexture(chrono.GetDataFile("concrete.jpg"))
        visual_asset.material_list.append(vis_mat)


        # Create and initialize the powertrain system
        self.powertrain = veh.SimplePowertrain(self.simplepowertrain_file)
        self.vehicle.InitializePowertrain(self.powertrain)

        # Create and initialize the tires
        for axle in self.vehicle.GetAxles():
            tireL = veh.RigidTire(self.rigidtire_file)
            self.vehicle.InitializeTire(
                tireL, axle.m_wheels[0], veh.VisualizationType_MESH)
            tireR = veh.RigidTire(self.rigidtire_file)
            self.vehicle.InitializeTire(
                tireR, axle.m_wheels[1], veh.VisualizationType_MESH)


        # create obstacles
        self.boxes = []
        for i in range(2):
            box = chrono.ChBodyEasyBox(1.5, 1.5, 10, 1000, True, True)
            box.SetPos(chrono.ChVectorD(50 + 25*i,(np.random.rand(1)-0.5)*13, 5.05))
            box.SetBodyFixed(True)
            box_asset = box.GetAssets()[0]
            visual_asset = chrono.CastToChVisualization(box_asset)

            vis_mat = chrono.ChVisualMaterial()
            vis_mat.SetAmbientColor(chrono.ChVectorF(0, 0, 0))
            vis_mat.SetDiffuseColor(chrono.ChVectorF(.2, .2, .9))
            vis_mat.SetSpecularColor(chrono.ChVectorF(.9, .9, .9))

            visual_asset.material_list.append(vis_mat)
            self.boxes.append(box)
            self.system.Add(box)

        # -------------
        # Create driver
        # -------------
        self.driver = Driver(self.vehicle)
        # Set the time response for steering and throttle inputs.
        # NOTE: this is not exact, since we do not render quite at the specified FPS.
        steering_time = 1.0
        # time to go from 0 to +1 (or from 0 to -1)
        throttle_time = 1.0
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
            chrono.ChFrameD(chrono.ChVectorD(1, 0, .875)),
            # offset pose
            320,  # number of horizontal samples
            180,  # number of vertical channels
            chrono.CH_C_PI / 3,  # horizontal field of view
            (180 / 320) * chrono.CH_C_PI / 3.  # vertical field of view
        )
        self.camera.SetName("Camera Sensor")
        self.manager.AddSensor(self.camera)

        # -----------------------------------------------------------------
        # Create a filter graph for post-processing the data from the lidar
        # -----------------------------------------------------------------

        self.camera.FilterList().append(sens.ChFilterRGBA8Access())
        
        self.step_number = 0
        self.isdone = False
        return self.get_ob()

    def step(self, ac):
        self.ac = ac.reshape((-1,))
        # Collect output data from modules (for inter-module communication)

        for i in range(round(1/self.control_frequency*self.timestep)):
            self.driver_inputs = self.driver.GetInputs()
            # Update modules (process inputs from other modules)
            time = self.system.GetChTime()
            self.driver.Synchronize(time)
            self.vehicle.Synchronize(time, self.driver_inputs, self.terrain)
            self.terrain.Synchronize(time)
            self.driver.SetTargetThrottle(self.ac[0,])
            self.driver.SetTargetSteering(self.ac[1,])
            self.driver.SetTargetBraking(self.ac[2,])

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
        #TODO self.camera input

        return np.asarray(self.state)

    def calc_rew(self, pos):
        dist_coeff = 10
        time_cost = -0.1
        progress = self.calc_progress()
        rew = dist_coeff*progress + time_cost*self.system.GetChTime()
        return rew

    def is_done(self, pos):
        if self.system.GetChTime() > self.timeend:
            self.isdone = True
        elif self.chassis_body.GetPos().z < -1 or self.collision:
            self.rew += -1000
            self.isdone = True

        elif self.chassis_body.GetPos().x > self.Xtarg :
            self.rew += 1000
            self.isdone = True


    def render(self):
        if not self.render_setup:
            """
            self.myapplication = veh.ChVehicleIrrApp(self.vehicle)
            self.myapplication.SetHUDLocation(500, 20)
            self.myapplication.SetSkyBox()
            self.myapplication.AddTypicalLogo()
            self.myapplication.AddTypicalLights(chronoirr.vector3df(-150., -150., 200.), chronoirr.vector3df(-150., 150., 200.), 100,
                                 100)
            self.myapplication.AddTypicalLights(chronoirr.vector3df(150., -150., 200.), chronoirr.vector3df(150., 150., 200.), 100,
                                 100)
            self.myapplication.EnableGrid(False)
            self.myapplication.SetChaseself.camera(self.trackPoint, 6.0, 0.5)

            self.myapplication.SetTimestep(self.timestep)
            # ---------------------------------------------------------------------
            #
            #  Create an Irrlicht application to visualize the system
            #
            # ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
            # in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
            # If you need a finer control on which item really needs a visualization proxy
            # Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

            self.myapplication.AssetBindAll()

            # ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
            # that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

            self.myapplication.AssetUpdateAll()
            self.myapplication.AddTypicalSky(chrono.GetChronoDataPath() + '/skybox/')
            self.myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + '/logo_pychrono_alpha.png')
            """
            vis_camera = sens.ChCameraSensor(
                self.chassis_body,  # body camera is attached to
                30,  # scanning rate in Hz
                chrono.ChFrameD(chrono.ChVectorD(-8, 0, 3), chrono.Q_from_AngAxis(chrono.CH_C_PI / 6, chrono.ChVectorD(0, 1, 0))),
                # offset pose
                1280,  # number of horizontal samples
                720,  # number of vertical channels
                chrono.CH_C_PI / 3,  # horizontal field of view
                (720/1280) * chrono.CH_C_PI / 3.  # vertical field of view
            )
            vis_camera.SetName("Camera Sensor")
            self.manager.AddSensor(vis_camera)

            # -----------------------------------------------------------------
            # Create a filter graph for post-processing the data from the lidar
            # -----------------------------------------------------------------


            self.camera.FilterList().append(sens.ChFilterVisualize("RGB Camera"))
            vis_camera.FilterList().append(sens.ChFilterVisualize("Visualization Camera"))
            self.render_setup = True
            
        pass
        """
        self.myapplication.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192))
        self.myapplication.DrawAll()
        self.myapplication.EndScene()
        self.step_number += 1
        self.myapplication.Synchronize("", self.driver_inputs)
        """
    def calc_progress(self):
        d = np.linalg.norm([self.Ytarg - self.chassis_body.GetPos().y, self.Xtarg - self.chassis_body.GetPos().x])
        progress = -(d - self.d_old) / self.timestep
        self.d_old = d
        return progress

    def close(self):
        del self

    def ScreenCapture(self, interval):
        try:
            self.myapplication.SetVideoframeSave(True)
            self.myapplication.SetVideoframeSaveInterval(interval)
        except:
            print("No ChIrrApp found. Cannot save video frames.")


    def __del__(self):
        if self.render:
            if hasattr(self, 'app'):
                self.myapplication.GetDevice().closeDevice()
            print("Destructor called, Device deleted.")
        else:
            print("Destructor called, No device to delete.")
