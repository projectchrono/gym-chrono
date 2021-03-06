#-------------------------------------------------------------------------------
# Name:        modulo1
# Purpose:
#
# Author:      Simone Benatti
#
# Created:     1/1/2019
#-------------------------------------------------------------------------------
#!/usr/bin/env python

def main():
    pass

if __name__ == '__main__':
    main()


import os, sys
import math
import numpy as np
#import sys, getopt
import pychrono as chrono
from gym_chrono.envs.ChronoBase import  ChronoBaseEnv
from gym import spaces
try:
   from pychrono import irrlicht as chronoirr
except:
   print('Could not import ChronoIrrlicht')
     
# ---------------------------------------------------------------------
#
# Parse command-line parameters
class ChronoHexapod(ChronoBaseEnv):
       def __init__(self):
              ChronoBaseEnv.__init__(self)
              # Set to False to avoid vis shapes loading. Once you do this, you can no longer render until this is re set to True
              self.animate = True
              
              low = np.full(53, -1000)
              high = np.full(53, 1000)
              self.observation_space = spaces.Box(low, high, dtype=np.float32)
              self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(18,), dtype=np.float32)
              
              self.Xtarg = 0.0
              self.Ztarg = 1000.0
              self.d_old = np.linalg.norm(self.Xtarg + self.Ztarg)
              
              #self.d_old = np.linalg.norm(self.Xtarg + self.Ytarg)
              self.hexapod_sys = chrono.ChSystemNSC()
              chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0001)
              chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0001)
              #hexapod_sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) # precise, more slow

              self.info =  {"timeout": 3200.0}
              if self.animate:
                  m_filename = "hexapod"
              else:
                  m_filename = "hexapod_novis"
              self.timestep = 0.005
              m_length = 1.0
              self.my_material = chrono.ChMaterialSurfaceNSC()
              self.my_material.SetFriction(0.5)
              self.my_material.SetDampingF(0.2)
              self.my_material.SetCompliance (0.0000001)
              self.my_material.SetComplianceT(0.0000001)
              #self.my_material.SetCompliance (0.0005)
              #self.my_material.SetComplianceT(0.0005)
              #m_visualization = "irrlicht" 
              print ("  file to load is ", m_filename)
              print ("  timestep is ", self.timestep)
              print ("  length is ", m_length)
              print ("  data path for fonts etc.: ", chrono.GetChronoDataPath())
              # ---------------------------------------------------------------------
              #
              #  load the file generated by the SolidWorks CAD plugin
              #  and add it to the ChSystem.
              # Remove the trailing .py and add / in case of file without ./
              #m_absfilename = os.path.abspath(m_filename)
              #m_modulename = os.path.splitext(m_absfilename)[0]

              print ("Loading C::E scene...");
              dir_path = os.path.dirname(os.path.realpath(__file__))
              self.fpath = os.path.join(dir_path, m_filename)
              #exported_items = chrono.ImportSolidWorksSystem(self.fpath)
              #self.exported_items = chrono.ImportSolidWorksSystem(m_modulename)
              
              print ("...loading done!");
              # Print exported items
              #for my_item in exported_items:
              	#print (my_item.GetName())		
              # Optionally set some solver parameters.
              #self.hexapod_sys.SetMaxPenetrationRecoverySpeed(1.00)
              solver = chrono.ChSolverBB()
              self.hexapod_sys.SetSolver(solver)
              solver.SetMaxIterations(600);
              solver.EnableWarmStart(True)
              self.hexapod_sys.Set_G_acc(chrono.ChVectorD(0,-9.8,0))
              """
              $$$$$$$$ FIND THE SW DEFINED CONSTRAINTS, GET THEIR self.frames AND GET RID OF EM $$$$$$$$ 
              """
              
              self.con_link = []
              self.coi_link = []
              self.hip_names = []
              self.femur_names = []
              self.tibia_names = []
              self.feet_names = []
              self.maxSpeed = [] 
              self.maxRot = [] 
              self.minRot = []
              Tmax = []
              
              for i in range (1,19):
                  self.con_link.append("Concentric"+str(i))
                  self.coi_link.append("Coincident"+str(i))
                  self.maxSpeed.append(354) # 59 RPM to deg/s
                  self.maxRot.append(90) #deg
                  self.minRot.append(-90) #deg
                  self.minRot.append(-90) #deg
                  Tmax.append(1.5) #deg
                  
              for i in range(1,7):
                  self.hip_names.append("Hip-"+str(i))
                  self.femur_names.append("Femur-"+str(i))
                  self.tibia_names.append("Tibia-"+str(i))
                  self.feet_names.append("Foot-"+str(i))

                     
              self.maxT = np.asarray(Tmax) 
              
       def reset(self):
    
              self.isdone = False
              self.hexapod_sys.Clear()              
              self.exported_items = chrono.ImportSolidWorksSystem(self.fpath)
              self.csys = []
              self.frames = []
              self.revs = []
              self.motors = []
              self.limits = []

              for con, coi in zip(self.con_link, self.coi_link):
                     indices = []
                     for i in range(len(self.exported_items)):
                            if con==self.exported_items[i].GetName() or coi==self.exported_items[i].GetName() : 
                                   indices.append(i) 
                     rev = self.exported_items[indices[0]]
                     af0 = rev.GetAssetsFrame()
                     # Revolute joints and ChLinkMotorRotation are z oriented, while parallel is x oriented. 
                     # Event though this Frame won't be used anymore is good practice to create a copy before editing its value.
                     af = chrono.ChFrameD(af0)
                     af.SetRot(af0.GetRot() % chrono.Q_ROTATE_X_TO_Z)
                     self.frames.append(af)
                     for i in reversed(indices): del self.exported_items[i]

              # ADD IMPORTED ITEMS TO THE SYSTEM
              for my_item in self.exported_items:
              	self.hexapod_sys.Add(my_item)
              
              
              """
              $$$$$$$$ FIND THE SW DEFINED CONSTRAINTS, GET THEIR MARKERS AND GET RID OF EM $$$$$$$$ 
              """
              self.hips = [self.hexapod_sys.SearchBody(name) for name in self.hip_names]
              self.femurs = [self.hexapod_sys.SearchBody(name) for name in self.femur_names]
              self.tibias = [self.hexapod_sys.SearchBody(name) for name in self.tibia_names]
              self.feet = [self.hexapod_sys.SearchBody(name) for name in self.feet_names]
              self.centralbody = self.hexapod_sys.SearchBody('Body-1')
              # Bodies are used to replace constraints and detect unwanted collision, so feet are excluded
              self.bodies =  [self.centralbody] + self.hips + self.femurs + self.tibias
              self.centralbody.SetBodyFixed(False)
              self.y0 = self.centralbody.GetPos().y
              
              """
              # SNIPPET FOR COLOR
              orange = chrono.ChColorAsset()
              orange.SetColor(chrono.ChColor(255/255,77/255,6/255))
              black = chrono.ChColorAsset()
              black.SetColor(chrono.ChColor(0,0,0))
              for body in self.bodies[:-1]:
                  assets = body.GetAssets()
                  for ast in assets:
                      ass_lev = chrono.CastToChAssetLevel(ast)
                      ass_lev.GetAssets().push_back(orange) 
                      
              assets = self.hand.GetAssets()
              for ast in assets:
                      ass_lev = chrono.CastToChAssetLevel(ast)
                      ass_lev.GetAssets().push_back(black) 
              """


              
              for i in range(len(self.con_link)):
                     revolute = chrono.ChLinkLockRevolute() 
                     cs = chrono.ChCoordsysD(self.frames[i].GetPos(), self.frames[i].GetRot())
                     self.csys.append(cs)
                     if i<6:
                         j = 0
                     else:
                         j = i-5
                     revolute.Initialize(self.bodies[j], self.bodies[i+1], self.csys[i])
                     self.revs.append(revolute)
                     self.hexapod_sys.Add(self.revs[i])
                     lim = self.revs[i].GetLimit_Rz()
                     self.limits.append(lim)
                     self.limits[i].SetActive(True)
                     self.limits[i].SetMin(self.minRot[i]*(math.pi/180))
                     self.limits[i].SetMax(self.maxRot[i]*(math.pi/180))
                     m = chrono.ChLinkMotorRotationTorque() 
                     m.SetSpindleConstraint(False, False, False, False, False)
                     m.Initialize(self.bodies[j], self.bodies[i+1], self.frames[i])
                     self.motors.append(m)
                     self.hexapod_sys.Add(self.motors[i])

              self.body_floor = chrono.ChBody()
              self.body_floor.SetBodyFixed(True)
              self.body_floor.SetPos(chrono.ChVectorD(0, -1-0.128-0.0045, 10 ))
              
              # Floor Collision.
              self.body_floor.GetCollisionModel().ClearModel()
              self.body_floor.GetCollisionModel().AddBox(self.my_material, 50, 1, 50, chrono.ChVectorD(0, 0, 0 ))
              self.body_floor.GetCollisionModel().BuildModel()
              self.body_floor.SetCollide(True)
              
              # Visualization shape
              body_floor_shape = chrono.ChBoxShape()
              body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(4, 1, 15)
              body_floor_shape.SetColor(chrono.ChColor(0.4,0.4,0.5))
              self.body_floor.GetAssets().push_back(body_floor_shape)
              body_floor_texture = chrono.ChTexture()
              texpath = os.path.join(chrono.GetChronoDataPath(), 'concrete.jpg')
              body_floor_texture.SetTextureFilename(texpath)
              self.body_floor.GetAssets().push_back(body_floor_texture)     
              self.hexapod_sys.Add(self.body_floor)

              
              self.numsteps= 0

              if (self.render_setup):
                     self.myapplication.AssetBindAll()
                     self.myapplication.AssetUpdateAll()	
              return self.get_ob()

       def step(self, ac):
              #posbefore = self.body_abdomen.GetPos().x
              self.numsteps += 1
              self.ac = ac.reshape((-1,))
              torques = np.multiply(self.ac, self.maxT)
              for i, t in enumerate(torques):
                            if self.revs[i].GetRelWvel().z > self.maxSpeed[i]*(math.pi/180) and torques[i] > 0: 
                                   torques[i] = 0
                            if self.revs[i].GetRelWvel().z < -self.maxSpeed[i]*(math.pi/180) and torques[i] < 0: 
                                   torques[i] = 0
              for m, t in zip(self.motors, torques): m.SetTorqueFunction(chrono.ChFunction_Const(float(t)))
              
              self.hexapod_sys.DoStepDynamics(self.timestep)
              
              obs= self.get_ob()
              rew = self.calc_rew()    
              
              self.is_done()
              return obs, rew, self.isdone, self.info              
       
       
       def get_ob(self):
       
              self.q_mot   = np.zeros([18,])
              self.q_dot_mot   = np.zeros([18,])
              joint_at_limit   = np.asarray([])
              for i, r in enumerate(self.revs): 
                     self.q_mot[i] = r.GetRelAngle()
                     self.q_dot_mot[i] = r.GetRelWvel().z
                     joint_at_limit = np.append(joint_at_limit,  [ r.GetLimit_Rz().GetMax()   < self.q_mot[i]   or r.GetLimit_Rz().GetMin()   > self.q_mot[i] ] )
              self.joint_at_limit = np.count_nonzero(np.abs(joint_at_limit))
              rotquat = self.centralbody.GetRot()
              self.rotquat= np.asarray([rotquat.e0, rotquat.e1, rotquat.e2, rotquat.e3])
              speedvec = self.centralbody.GetRot().RotateBack(self.centralbody.GetPos_dt())
              self.speed= np.asarray([speedvec.x, speedvec.y, speedvec.z])
              omegavec = self.centralbody.GetWvel_loc()
              self.omega = np.asarray([omegavec.x, omegavec.y, omegavec.z])
              feet_contact = np.asarray([foot.GetContactForce().Length() for foot in self.feet])
              feet_contact = np.clip(feet_contact , 0, 5)
              
              return np.concatenate ([[self.centralbody.GetPos().y], self.rotquat, self.speed, self.omega, self.q_mot,  self.q_dot_mot, feet_contact])
       
       def calc_rew(self):
       
              electricity_cost     = -0.1    # cost for using motors -- this parameter should be carefully tuned against reward for making progress, other values less improtant
              #stall_torque_cost    = -0.1    # cost for running electric current through a motor even at zero rotational speed, small
        
              joints_at_limit_cost = -0.8    # discourage stuck joints
              
              progress_bonus = 5
              power_cost  = electricity_cost  * float(np.abs(self.ac*self.q_dot_mot).mean())  # let's assume we have DC motor with controller, and reverse current braking. BTW this is the formula of motor power
              
              joints_limit = joints_at_limit_cost * self.joint_at_limit
              collision = 0
              for body in self.bodies:
                   collision += body.GetContactForce().Length()
              self.alive_bonus =  +1 if collision == 0 else -1
              progress = progress_bonus * self.calc_progress()
              # height bonus
              h_rew = 2*(self.centralbody.GetPos().y - (self.y0))
              rew = progress + self.alive_bonus + (power_cost) + (joints_limit) + h_rew
              return rew


       def calc_progress(self):
              d = np.linalg.norm( [self.Ztarg - self.centralbody.GetPos().z, self.Xtarg - self.centralbody.GetPos().x] )
              progress = -(d - self.d_old )/self.timestep
              self.d_old = d
              return progress                  
       def is_done(self):
       
              if ( self.alive_bonus < 0 or self.centralbody.GetPos().z > 49 or self.centralbody.GetPos().x > 49 or self.numsteps *self.timestep>16):
                            self.isdone = True
       def get_prog(self):
          
              return np.array([[self.numsteps *self.timestep],[self.centralbody.GetPos().y]])


                     
       def render(self):
             if not self.animate :
                 print('It seems that for efficiency reasons visualization has been turned OFF. To render the simulation set self.animate = True in ChronoHexapod.py')
                 sys.exit(1)
             if not self.render_setup :
                     self.myapplication = chronoirr.ChIrrApp(self.hexapod_sys, 'Test', chronoirr.dimension2du(1280,720))
                     self.myapplication.AddShadowAll()
                     self.myapplication.SetStepManage(True)
                     self.myapplication.SetTimestep(self.timestep)
                     self.myapplication.AddTypicalSky(chrono.GetChronoDataPath() + '/skybox/')
                     self.myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + '/logo_pychrono_alpha.png')
                     self.myapplication.AddTypicalCamera(chronoirr.vector3df(1,1,1),chronoirr.vector3df(0.0,0.0,0.0))
                     self.myapplication.AddTypicalLights()               # angle of FOV              # angle of FOV
                     self.myapplication.AssetBindAll()
                     self.myapplication.AssetUpdateAll()
                     self.render_setup = True
             #self.myapplication.GetSceneManager().getActiveCamera().setPosition(chronoirr.vector3df(self.centralbody.GetPos().x - 0.75 , 0.4, self.centralbody.GetPos().z + 0.25))
             #self.myapplication.GetSceneManager().getActiveCamera().setTarget(chronoirr.vector3df(self.centralbody.GetPos().x , 0.25, self.centralbody.GetPos().z))
             self.myapplication.GetDevice().run()
             self.myapplication.BeginScene()
             self.myapplication.DrawAll()
             self.myapplication.EndScene()
