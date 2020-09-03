import pychrono as chrono
import numpy as np

import re

surface_mat = chrono.ChMaterialSurfaceNSC()
surface_mat.SetFriction(0.9)
surface_mat.SetRestitution(0.01)

class Asset:
    def __init__(self, filename, scale_range=(1,1)):
        self.filename = filename
        self.scale_range = scale_range
        self.ready = False

        self.mesh = chrono.ChTriangleMeshConnected()
        self.mesh.LoadWavefrontMesh(chrono.GetChronoDataFile(self.filename), True, True)

        self.shape = chrono.ChTriangleMeshShape()
        self.shape.SetMesh(self.mesh)
        self.shape.SetStatic(True)

        self.body = chrono.ChBody()
        self.body.AddAsset(self.shape)
        self.body.SetCollide(True)
        self.body.SetBodyFixed(True)

        self.body.GetCollisionModel().ClearModel()
        self.body.GetCollisionModel().AddTriangleMesh(surface_mat, self.mesh, False, False)
        self.body.GetCollisionModel().BuildModel()

        self.scale = 1

    def CreateCollisionModel(self):
        self.body.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(0)
        self.body.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(3)
        self.body.GetCollisionModel().SetFamily(3)

    def Transform(self):
        self.mesh.Transform(chrono.ChVectorD(0,0,0), chrono.ChMatrix33D(self.scale))

    @property
    def pos(self):
        return self.body.GetPos()

    @pos.setter
    def pos(self, p):
        self.body.SetPos(p)
        self.ready = True

    @property
    def rot(self):
        return self.body.GetRot()

    @rot.setter
    def rot(self, r):
        self.body.SetRot(r)

    def GetName(self):
        return re.search('sensor/offroad/(.*).obj', self.filename).group(1)

    def Copy(self):
        return Asset(self.filename, self.scale_range)

class AssetHandler:
    def __init__(self, mat=None, b1=0, b2=0, r1=0, r2=0, r3=0, r4=0, r5=0, t1=0, t2=0, t3=0, c=0):
        if mat is not None:
            global surface_mat
            surface_mat = mat
        
        self.assets = []
        for _ in range(b1):
            self.assets.append(Asset("sensor/offroad/bush1.obj", (1, 2)))
        for _ in range(b2):
            self.assets.append(Asset("sensor/offroad/bush2.obj", (0.5, 1.5)))
        for _ in range(r1):
            self.assets.append(Asset("sensor/offroad/rock1.obj", (0.5, 1)))
        for _ in range(r2):
            self.assets.append(Asset("sensor/offroad/rock2.obj", (0.5, 1.1)))
        for _ in range(r3):
            self.assets.append(Asset("sensor/offroad/rock3.obj", (0.5, 1.1)))
        for _ in range(r4):
            self.assets.append(Asset("sensor/offroad/rock4.obj", (0.5, 1.1)))
        for _ in range(r5):
            self.assets.append(Asset("sensor/offroad/rock5.obj", (0.5, 1.1)))
        for _ in range(t1):
            self.assets.append(Asset("sensor/offroad/tree1.obj", (0.5, 2)))
        for _ in range(t2):
            self.assets.append(Asset("sensor/offroad/tree2.obj", (0.15, .5)))
        for _ in range(t3):
            self.assets.append(Asset("sensor/offroad/tree3.obj", (1, 1)))
        for _ in range(c):
            self.assets.append(Asset("sensor/offroad/cottage.obj", (1, 1)))

    def RandomlyPositionAssets(self, system, initLoc, finalLoc, terrain, terrain_length, terrain_width, should_scale=False):
        diag_obs = 5
        for i in range(diag_obs):
            x = np.linspace(initLoc.x, finalLoc.x, diag_obs + 2)[1:-1]
            y = np.linspace(initLoc.y, finalLoc.y, diag_obs + 2)[1:-1]
            pos = chrono.ChVectorD(x[i], y[i], 0)
            pos.z = terrain.GetHeight(pos)

            rot = chrono.Q_from_AngZ(np.random.uniform(0, np.pi))

            offset = chrono.ChVectorD(pos.y, -pos.x, 0)
            offset = offset.GetNormalized() * (np.random.random() - 0.5) * 20

            rand_asset = np.random.choice(self.assets).Copy()
            rand_asset.pos = pos + offset
            rand_asset.rot = rot
            if should_scale:
                rand_asset.scale = np.random.uniform(rand_asset.scale_range[0], rand_asset.scale_range[1])

            self.assets.append(rand_asset)

        for i, asset in enumerate(self.assets[:-diag_obs]):
            success = True
            for i in range(101):
                if i == 100:
                    success = False
                    break

                pos = self.GenerateRandomPosition(terrain, terrain_length, terrain_width)
                scale = 1
                if should_scale:
                    scale = np.random.uniform(asset.scale_range[0], asset.scale_range[1])

                if (pos - initLoc).Length() < 15 or (pos - finalLoc).Length() < 15:
                    continue

                closest_asset = min(self.assets, key=lambda x: (x.pos - pos).Length())
                overlap = 0
                if (closest_asset.pos - pos).Length() < scale + closest_asset.scale + overlap:
                    continue

                break

            if not success:
                continue

            rot = chrono.Q_from_AngZ(np.random.uniform(0, np.pi))

            asset.pos = pos
            asset.rot = rot
            asset.scale = scale

        for asset in self.assets:
            system.Add(asset.body)
            asset.CreateCollisionModel()

    def GenerateRandomPosition(self, terrain, terrain_length, terrain_width):
        x = (np.random.rand() - 0.5) * terrain_length
        y = (np.random.rand() - 0.5) * terrain_width
        pos = chrono.ChVectorD(x, y, 0)
        pos.z = terrain.GetHeight(pos)
        return pos

    def ResetAssets(self):
        for asset in self.assets:
            asset.Transform()

    def Write(self, filename='obstacles.txt', arr=None):
        if arr == None:
            arr = self.assets
        with open(filename, 'w') as file:
            for asset in arr:
                if not asset.ready:
                    continue
                pos = asset.pos
                rot = asset.rot
                name = asset.GetName()
                file.write(f'{name},{pos.x},{pos.y},{pos.z},{rot.Q_to_Euler123().z},{asset.scale}\n')

    def GetContactedAssets(self):
        ca = []
        for asset in self.assets:
            if not asset.ready:
                continue
            if asset.body.GetContactForce().Length():
                ca.append(asset)

        return ca
    
    def AddAsset(self,asset):
        self.assets.append(asset)
    
    def __getitem__(self,i):
        return self.assets[i]
