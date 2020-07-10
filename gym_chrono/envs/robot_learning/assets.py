import pychrono as chrono
import pychrono.sensor as sens
import random

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

    def RandomlyPositionAssets(self, system, vehicle_pos, goal_pos, terrain, length, width, should_scale=True):
        for asset in self.assets:
            for _ in range(asset.num):
                # Check if position is too close to another asset, vehicle or goal
                while True:
                    # Calculate random transformation values
                    pos = self.CalcRandomPose(terrain, length, width, offset=-random.random()*.5)
                    if should_scale:
                        scale = self.map(random.random(), asset.min_scale, asset.max_scale)
                    else:
                        scale = 1
                    threshold = asset.mesh.bounding_box.Length() * scale / 2
                    if (pos - vehicle_pos).Length() < 15:
                        continue
                    if len(self.positions) == 0:
                        break
                    min_pos = min(self.positions, key=lambda x: (x - pos).Length())
                    if  (pos - min_pos).Length() > threshold and (pos - goal_pos).Length() > 15:
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
        x = random.randint(int(-length/2), int(length/2))
        y = random.randint(int(-width/2), int(width/2))
        z = terrain.GetHeight(chrono.ChVectorD(x, y, 0)) + offset
        return chrono.ChVectorD(x,y,z)

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
