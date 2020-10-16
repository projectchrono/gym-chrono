import pychrono as chrono

import numpy as np
import math as m
from scipy.interpolate import splprep, splev

class BezierPath(chrono.ChBezierCurve):
    def __init__(self, points, t0=0.00):
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
        
