import pychrono as chrono
import numpy as np

class Asset:
    def __init__(self, filename, scale_range):

        self.mesh = chrono.ChTriangleMeshConnected()
        self.mesh.LoadWavefrontMesh(chrono.GetChronoDataFile(filename), True, True)

        self.shape = chrono.ChTriangleMeshShape()
        self.shape.SetMesh(self.mesh)
        self.shape.SetStatic(True)

        self.body = chrono.ChBody()
        self.body.AddAsset(self.shape)
        self.body.SetCollide(True)
        self.body.SetBodyFixed(True)


        surface_mat = chrono.ChMaterialSurfaceNSC()
        surface_mat.SetFriction(0.9)
        surface_mat.SetRestitution(0.01)

        self.body.GetCollisionModel().ClearModel()
        self.body.GetCollisionModel().AddTriangleMesh(surface_mat, self.mesh, False, False)
        self.body.GetCollisionModel().BuildModel()

        self.scale_range = scale_range

        self.ready = False

    def CreateCollisionModel(self):
        self.body.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(0)
        self.body.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(1)
        self.body.GetCollisionModel().SetFamily(1)

    @property
    def pos(self):
        return self.body.GetPos()

    @pos.setter
    def pos(self, p):
        self.body.SetPos(p)
        self.ready = True


class AssetHandler:
    def __init__(self, b1=0, b2=0, r1=0, r2=0, r3=0, r4=0, r5=0, t1=0, t2=0, t3=0, c=0):
        self.assets = []
        for _ in range(b1):
            self.assets.append(Asset("sensor/offroad/bush.obj", (0.5, 1.5)))
        for _ in range(b2):
            self.assets.append(Asset("sensor/offroad/bush2.obj", (0.5, 1.5)))
        for _ in range(r1):
            self.assets.append(Asset("sensor/offroad/rock1.obj", (0.25, 1)))
        for _ in range(r2):
            self.assets.append(Asset("sensor/offroad/rock2.obj", (0.25, .75)))
        for _ in range(r3):
            self.assets.append(Asset("sensor/offroad/rock3.obj", (0.25, .75)))
        for _ in range(r4):
            self.assets.append(Asset("sensor/offroad/rock4.obj", (0.25, .75)))
        for _ in range(r5):
            self.assets.append(Asset("sensor/offroad/rock5.obj", (0.25, .75)))
        for _ in range(t1):
            self.assets.append(Asset("sensor/offroad/tree1.obj", (0.5, 2)))
        for _ in range(t2):
            self.assets.append(Asset("sensor/offroad/tree2.obj", (0.15, .5)))
        for _ in range(t3):
            self.assets.append(Asset("sensor/offroad/tree3.obj", (5, 5)))
        for _ in range(c):
            self.assets.append(Asset("sensor/offroad/cottage.obj", (1, 1)))

    def RandomlyPositionAssets(self, system, initLoc, finalLoc, terrain, terrain_length, terrain_width):

        for i, asset in enumerate(self.assets):
            diag_obs = 3
            if i < diag_obs:
                x = np.linspace(initLoc.x, finalLoc.x, diag_obs + 2)[1:-1]
                y = np.linspace(initLoc.y, finalLoc.y, diag_obs + 2)[1:-1]
                pos = chrono.ChVectorD(x[i], y[i], 0)
                pos.z = terrain.GetHeight(pos)
            else:
                success = True
                for i in range(101):
                    if i == 100:
                        success = False
                        break

                    pos = self.GenerateRandomPosition(terrain, terrain_length, terrain_width)

                    if (pos - initLoc).Length() < 15 or (pos - finalLoc).Length() < 15:
                        continue

                    closest_asset = min(self.assets, key=lambda x: (x.pos - pos).Length())
                    if (closest_asset.pos - pos).Length() < 3:
                        continue

                    break

                if not success:
                    continue

            asset.pos = pos

            system.Add(asset.body)
            asset.CreateCollisionModel()

    def GenerateRandomPosition(self, terrain, terrain_length, terrain_width):
        x = (np.random.rand() - 0.5) * terrain_length
        y = (np.random.rand() - 0.5) * terrain_width
        pos = chrono.ChVectorD(x, y, 0)
        pos.z = terrain.GetHeight(pos)
        return pos

    def Write(self, filename='obstacles.txt', arr=None):
        if arr == None:
            arr = self.assets
        with open(filename, 'w') as file:
            for asset in arr:
                if not asset.ready:
                    continue
                pos = asset.pos
                file.write(f'{pos.x},{pos.y},{pos.z}\n')

    def GetContactedAssets(self):
        ca = []
        for asset in self.assets:
            if not asset.ready:
                continue
            if asset.body.GetContactForce().Length():
                ca.append(asset)

        return ca
