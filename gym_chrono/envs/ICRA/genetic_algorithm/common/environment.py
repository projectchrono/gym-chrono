import numpy as np

from common.geometry import Point, Polygon, Rectangle
from shapely.geometry import MultiPolygon

import os

obstacle_path = os.path.join(os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir)), 'obstacles', '')
mesh_filenames = ['rock1', 'rock2', 'rock3', 'rock4', 'rock5', 'bush1', 'bush2']

class Grid:
    def __init__(self, inputs):
        self.minimum = inputs.grid_minimum
        self.maximum = inputs.grid_maximum
        self.first = inputs.start
        self.final = inputs.goal
        self.width = self.maximum.x - self.minimum.x
        self.height = self.maximum.y - self.minimum.y

    def generate_boundaries(self, dimension):
        return [
            Rectangle(self.width, dimension, 0.0, Point(self.minimum.x, self.maximum.y)),
            Rectangle(self.width, dimension, 0.0, Point(self.minimum.x, self.minimum.y - dimension)),
            Rectangle(dimension, self.height + dimension * 2.0, 0.0, Point(self.maximum.x, self.minimum.y - dimension)),
            Rectangle(dimension, self.height + dimension * 2.0, 0.0, Point(self.minimum.x - dimension, self.minimum.y - dimension))
        ]

    def random_point(self, offset=Point(0,0)):
        return Point(
            np.random.randint(self.minimum.x+offset.x, self.maximum.x-offset.x+1),
            np.random.randint(self.minimum.y+offset.y, self.maximum.y-offset.y+1),
        )
    
    def mutate_point(self, point):
        return Point(
            point.x + np.random.randint(-2,2), 
            point.y + np.random.randint(-2,2)
            )
    
    def random_mesh(self):
        return obstacle_path + np.random.choice(mesh_filenames) + ".txt"

    def generate_obstacles(self, count, theta):
        obstacles = []
        for _ in range(count):
            obstacle = Polygon(self.random_mesh(), self.random_point(), theta)
            while self.first.intersects(obstacle) or self.final.intersects(obstacle):
                obstacle = Polygon(self.random_mesh(), self.random_point(), theta)
            obstacles.append(obstacle)
        
        return obstacles


def generate(inputs):

    grid = Grid(inputs)

    if inputs.obstacles is None:
        obstacles = grid.generate_obstacles(inputs.obstacle_count, inputs.obstacle_theta)
    else:
        obstacles = []
        for o in inputs.obstacles:
            filename = obstacle_path + o.GetName() + '.txt'
            obstacle = Polygon(filename, o.pos, o.rot.Q_to_Euler123().z, o.scale)
            obstacles.append(obstacle)

    obstacles = MultiPolygon(obstacles).buffer(0)

    bounds = obstacles[0].bounds
    dimension = max(bounds[2] - bounds[0], bounds[3] - bounds[1])
    for o in obstacles[1:]:
        bounds = obstacles[0].bounds
        dim = max(bounds[2] - bounds[0], bounds[3] - bounds[1]) 
        dimension = dim if dim < dimension else dimension
    boundaries = grid.generate_boundaries(dimension)

    return grid, boundaries, obstacles
