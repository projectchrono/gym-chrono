import pychrono as chrono
from random import randint


class Asset:
    def __init__(self, filename, scale_range):

        self.mesh = chrono.ChTriangleMeshConnected()
        self.mesh.LoadWavefrontMesh(
            chrono.GetChronoDataFile(filename), False, True)

        self.shape = chrono.ChTriangleMeshShape()
        self.shape.SetMesh(self.mesh)
        self.shape.SetStatic(True)

        self.body = chrono.ChBody()
        self.body.AddAsset(self.shape)
        self.body.SetCollide(False)
        self.body.SetBodyFixed(True)

        self.scale_range = scale_range

    @property
    def pos(self):
        return self.body.GetPos()

    @pos.setter
    def pos(self, p):
        self.body.SetPos(p)


class AssetHandler:
    def __init__(self, b1=0, b2=0, r1=0, r2=0, r3=0, r4=0, r5=0, t1=0, t2=0, t3=0, c=0):
        self.assets = []
        for _ in b1:
            self.assets.append(Asset("sensor/offroad/bush.obj", (0.5, 1.5)))
        for _ in b2:
            self.assets.append(Asset("sensor/offroad/bush2.obj", (0.5, 1.5)))
        for _ in r1:
            self.assets.append(Asset("sensor/offroad/rock1.obj", (0.25, 1)))
        for _ in r2:
            self.assets.append(Asset("sensor/offroad/rock2.obj", (0.25, .75)))
        for _ in r3:
            self.assets.append(Asset("sensor/offroad/rock3.obj", (0.25, .75)))
        for _ in r4:
            self.assets.append(Asset("sensor/offroad/rock4.obj", (0.25, .75)))
        for _ in r5:
            self.assets.append(Asset("sensor/offroad/rock5.obj", (0.25, .75)))
        for _ in t1:
            self.assets.append(Asset("sensor/offroad/tree1.obj", (0.5, 2)))
        for _ in t2:
            self.assets.append(Asset("sensor/offroad/tree2.obj", (0.15, .5)))
        for _ in t3:
            self.assets.append(Asset("sensor/offroad/tree3.obj", (5, 5)))
        for _ in c:
            self.assets.append(Asset("sensor/offroad/cottage.obj", (1, 1)))

    def RandomlyPositionAssets(self, system, initLoc, finalLoc, terrain, terrain_length, terrain_width):

        for i, asset in enumerate(self.assets):
            diag_obs = 3
            if i < diag_obs:
                x = np.linspace(vehicle_pos.x, goal_pos.x, obs + 2)[1:-1]
                y = np.linspace(vehicle_pos.y, goal_pos.y, obs + 2)[1:-1]
                pos = chrono.ChVectorD(x, y, 0)
                pos.z = terrain.GetHeight(pos)

                asset.pos = pos

            for i in range(100):
                pos = GenerateRandomPosition(terrain, terrain_length, terrain_width)

                if (pos - initLoc).Length() < 15 or (pos - initLoc).Length() < 15:
                    continue

                closest_asset = min(
                    self.assets, key=lambda x: (x.pos - pos).Length())
                if (closest_asset.pos - pos).Length() < 10:
                    continue

            asset.pos = pos

    def GenerateRandomPosition(self, terrain, terrain_length, terrain_width):
        x = randint(int(-terrain_length / 2), int(terrain_length / 2))
        y = randint(int(-terrain_width / 2), int(terrain_width / 2))
        pos = chrono.ChVectorD(x, y, 0)
        pos.z = terrain.GetHeight(pos)
        return pos
