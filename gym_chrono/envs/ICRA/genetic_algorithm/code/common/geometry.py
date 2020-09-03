import numpy as np

from shapely.geometry import Point as SPoint
from shapely.geometry import Polygon as SPolygon
from shapely.geometry import LineString as SLine


class Point(SPoint):
    def __init__(self, x, y):
        super().__init__(x, y)
        self.center = (self.x, self.y)

    def scale(self, sx, sy):
        return Point(self.x * sx, self.y * sy)

    def rotate(self, theta):
        return Point(
            self.x * np.cos(theta) - self.y * np.sin(theta),
            self.x * np.sin(theta) + self.y * np.cos(theta)
        )

    def translate(self, point):
        return Point(self.x + point.x, self.y + point.y)


class Line(SLine):
    def __init__(self, first, final):
        super().__init__([first.center, final.center])
        self.first = first
        self.final = final
        self.dx = final.x - first.x
        self.dy = final.y - first.y
        self.theta = np.arctan2(self.dy, self.dx)

    def divide(self, d):
        ix = self.dx / (d+1)
        iy = self.dy / (d+1)
        points = []
        for i in range(1, d+1):
            points.append(Point(
                self.first.x+ix*i,
                self.first.y+iy*i
            ))
        points.append(self.final)
        return points


class Polygon(SPolygon):
    def __init__(self, sx, sy, theta, t):
        corners = [
            Point(0.0, 0.0).scale(sx, sy).rotate(theta).translate(t).center,
            Point(1.0, 0.0).scale(sx, sy).rotate(theta).translate(t).center,
            Point(1.0, 1.0).scale(sx, sy).rotate(theta).translate(t).center,
            Point(0.0, 1.0).scale(sx, sy).rotate(theta).translate(t).center
        ]
        super().__init__(corners)
        self.datum = corners[0]
        self.width = sx
        self.height = sy
        self.rotation = theta

    def angle(self):
        # convert from radians to degrees
        return self.rotation * 180.0 / np.pi
