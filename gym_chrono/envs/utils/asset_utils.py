import pychrono as chrono
try:
    import pychrono.sensor as sens
except:
    print('Could not import Chrono Sensor')


import random


class Asset():
    """"Class that initializes an asset"""

    def __init__(self, visual_shape_path, scale=None, collision_shape_path=None, bounding_box=None):
        if (scale == None):
            self.scale = 1
            self.scaled = False
        else:
            self.scale = scale
            self.scaled = False

        self.visual_shape_path = visual_shape_path
        self.collision_shape_path = collision_shape_path

        # Intialize a random material
        self.material = chrono.ChMaterialSurfaceNSC()
        # initialize the body
        self.body = chrono.ChBodyAuxRef()
        # set body as fixed
        self.body.SetBodyFixed(True)

        # Get the visual mesh
        visual_shape_obj = chrono.GetChronoDataFile(visual_shape_path)
        visual_mesh = chrono.ChTriangleMeshConnected(
        ).CreateFromWavefrontFile(visual_shape_obj, False, False)
        visual_mesh.Transform(chrono.ChVectorD(0, 0, 0),
                              chrono.ChMatrix33D(scale))
        # Add this mesh to the visual shape
        self.visual_shape = chrono.ChVisualShapeTriangleMesh()
        self.visual_shape.SetMesh(visual_mesh)
        # Add visual shape to the mesh body
        self.body.AddVisualShape(self.visual_shape)

        # Get the collision mesh
        collision_shape_obj = None
        if (collision_shape_path == None):
            # Just use the bounding box
            if (bounding_box == None):
                self.body.SetCollide(False)
                self.collide_flag = False
            else:
                # self.body.GetCollisionModel().ClearModel()
                size = bounding_box * scale
                material = chrono.ChMaterialSurfaceNSC()
                collision_shape = chrono.ChCollisionShapeBox(
                    material, size.x, size.y, 5)
                self.body.AddCollisionShape(collision_shape)
                self.body.SetCollide(True)
                self.collide_flag = True
        else:
            collision_shape_obj = chrono.GetChronoDataFile(
                collision_shape_path)
            collision_mesh = chrono.ChTriangleMeshConnected(
            ).CreateFromWavefrontFile(collision_shape_obj, False, False)
            collision_mesh.Transform(chrono.ChVectorD(0, 0, 0),
                                     chrono.ChMatrix33D(scale))
            collision_shape = chrono.ChCollisionShapeTriangleMesh(self.material, collision_mesh,
                                                                  True, True, chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))
            self.body.AddCollisionShape(collision_shape)
            # Update the collision model
            self.body.SetCollide(True)
            self.collide_flag = True

        self.collision_shape = collision_shape_obj
        self.bounding_box = bounding_box

        # Asset has a position and orientation which will be set by the simulation assets class
        self.pos = chrono.ChVectorD()
        self.ang = 0

    def UpdateAssetPosition(self, pos, ang):
        self.pos = pos
        self.ang = ang
        self.body.SetFrame_REF_to_abs(chrono.ChFrameD(
            pos, ang))

    # Create a copy constructor for the asset
    def Copy(self):
        """Returns a copy of the asset"""
        asset = Asset(self.visual_shape_path, self.scale,
                      self.collision_shape_path, self.bounding_box)
        return asset


class SimulationAssets():
    """Class that handles assets for the Gym Environment"""

    def __init__(self, system, terrain, length, width):
        self.system = system
        self.terrain = terrain
        self.length = length
        self.width = width
        self.assets_list = []
        self.positions = []

    def AddAsset(self, asset, number=1):
        """Numer of such asset to be added"""
        for _ in range(number):
            new_asset = asset.Copy()
            self.assets_list.append(new_asset)

    # Position assets relative to goal and chassis

    def RandomlyPositionAssets(self, goal_pos, chassis_body):
        """Randomly positions assets within the terrain"""
        for asset in self.assets_list:
            # Calculate random transformation values
            while True:
                pos = self.CalcRandomPose(
                    offset=-random.random() * .5)
                if (asset.bounding_box == None):
                    threshold = 3
                else:
                    threshold = asset.bounding_box.Length() * asset.scale
                if len(self.positions) == 0:
                    break
                min_pos = min(self.positions,
                              key=lambda x: (x - pos).Length())
                if (pos - chassis_body.GetPos()).Length() > 15 and (pos - min_pos).Length() > threshold and (
                        pos - goal_pos).Length() > 15:
                    break

            # Append the positon
            self.positions.append(pos)
            # Update asset positon
            asset.UpdateAssetPosition(pos, chrono.ChQuaternionD(1, 0, 0, 0))
            # Add asset to the system
            self.system.Add(asset.body)

    # Utility function to calculate random position
    def CalcRandomPose(self, offset=0):
        """
        Calculates random position within the terrain boundaries

        TODO: generate some rotation (quaternion) to have mesh lay flush with the terrain
        """
        x = random.randint(int(-self.length / 2), int(self.length / 2))
        y = random.randint(int(-self.width / 2), int(self.width / 2))
        z = self.terrain.GetHeight(chrono.ChVectorD(x, y, 0)) + offset
        return chrono.ChVectorD(x, y, z)

    def CheckContact(self, chassis_body, proper_collision=False):
        """Checks if the chassis is in contact with any asset"""
        # First check if the user wants to check for collision using mesh or bounding box

        if proper_collision:
            # Check for collision using the collision model
            for asset in self.assets_list:
                if (asset.body.GetContactForce().Length() > 0):
                    return 1
            return 0
        else:
            # Check for collision using the absolute position of the asset
            pos = chassis_body.GetPos()
            for asset_pos in self.positions:
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
