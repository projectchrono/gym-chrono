import pychrono as chrono

import numpy as np
import math as m
from scipy.interpolate import splprep, splev

from gym_chrono.envs.ICRA.genetic_algorithm.code.genetic_algorithm import create_path, main

class BezierPath(chrono.ChBezierCurve):
    def __init__(self, points, t0=0.05):
        super(BezierPath, self).__init__(points)

        self.current_t = t0

    # Update the progress on the path of the leader
    def Advance(self, delta_t):
        self.current_t += delta_t

    def getPoints(self):
        points = []
        for i in range(self.getNumPoints()):
            points.append(self.getPoint(i))
        return points

    def calc_i(self, t):
        par = np.clip(t, 0.0, 1.0)
        numIntervals = self.getNumPoints() - 1
        epar = par * numIntervals
        i = m.floor(par * numIntervals)
        i = np.clip(i, 0, numIntervals - 1)
        return i

    # Param-only derivative
    def par_evalD(self, t):
        par = np.clip(t, 0.0, 1.0)
        numIntervals = self.getNumPoints() - 1
        epar = par * numIntervals
        i = m.floor(par * numIntervals)
        i = np.clip(i, 0, numIntervals - 1)
        return self.evalD(int(i), epar - i)

    # Current positon and rotation of the leader vehicle chassis
    def GetPose(self, t):
        pos = self.eval(t)
        posD = self.par_evalD(t)
        alpha = m.atan2(posD.y, posD.x)
        rot = chrono.Q_from_AngZ(alpha)
        return pos, rot


def CreatePath(start, goal, assets):
    obs = []
    for asset in assets:
        pos = asset.pos
        obs.append([pos.x, pos.y])

    path = create_path(obs)

    # smooth path
    points = np.array([[p.x,p.y] for p in path.points])
    tck, u = splprep(points.T, s=0, per=0)
    u_new = np.linspace(u.min(), u.max(), 100)
    x,y = splev(u_new, tck, der=0)
    points = np.array(list(zip(x,y)))

    vec = chrono.vector_ChVectorD()
    for p in points:
        vec.push_back(chrono.ChVectorD(p[0], p[1], 0.25))
    
    
    return BezierPath(vec)
        
