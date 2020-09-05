from gym_chrono.envs.ICRA.assets import AssetHandler, Asset

class GhostLeaderHandler(AssetHandler):
    def __init__(self, surface_mat, num_leaders, interval, asset_filename):
        AssetHandler.__init__(surface_mat)

        self.num_leaders = num_leaders
        self.interval = interval

        self.asset_filename = asset_filename

    def Initialize(self, system, path):
        self.leaders = []
        self.path = path

        for i in range(self.num_leaders):
            leader = Asset(self.asset_filename)
            system.Add(leader.body)
            leader.CreateCollisionModel()

            self.leaders.append(leader)

        self.Update()

    def Update(self):
        t = self.path.current_t
        for i, leader in enumerate(self.leaders):
            pos, rot = self.path.GetPose(t + (i+1) * self.interval)

            leader.pos = pos
            leader.rot = rot

    def __getitem__(self, item):
        return self.leaders[item]