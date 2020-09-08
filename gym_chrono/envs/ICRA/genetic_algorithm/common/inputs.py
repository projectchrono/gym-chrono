from common.geometry import Point

class Inputs:
    def __init__(self, algorithm):
        self.verbose = False
        self.plot = False

        self.grid_minimum = Point(0.0, 0.0)
        self.grid_maximum = Point(20.0, 20.0)

        self.start = Point(self.grid_minimum.x + 2, self.grid_minimum.y + 2)
        self.goal = Point(self.grid_maximum.x - 2, self.grid_maximum.y - 2)

        self.obstacle_size = Point(1.0, 1.0)
        self.obstacle_count = 40
        self.obstacle_theta = 0.0
        self.obstacles = None

        if algorithm == 'genetic':
            self.population_count = 80
            self.interpolation = 8
            self.path_segments = 2
            self.curve_samples = 16
            self.mutation_chance = 0.04
            self.evolution_max = 10


    @staticmethod
    def parse(algorithm):
        import argparse

        parser = argparse.ArgumentParser()

        parser.add_argument(
            "-v", "--verbose", action="store_true",
            help="enables additional logging with extra information"
        )

        parser.add_argument(
            "-p", "--plot", action="store_true",
            help="enables plotting of the results"
        )

        parser.add_argument(
            "-lx", "--leftx", type=int,
            help="x-coordinate of the bottom-left point of the grid"
        )

        parser.add_argument(
            "-ly", "--lefty", type=int,
            help="y-coordinate of the bottom-left point of the grid"
        )

        parser.add_argument(
            "-rx", "--rightx", type=int,
            help="x-coordinate of the top-right point of the grid"
        )

        parser.add_argument(
            "-ry", "--righty", type=int,
            help="y-coordinate of the top-right point of the grid"
        )

        parser.add_argument(
            "-sx", "--startx", type=int,
            help="x-coordinate of the start point of the vehicle"
        )

        parser.add_argument(
            "-sy", "--starty", type=int,
            help="y-coordinate of the start point of the vehicle"
        )

        parser.add_argument(
            "-ex", "--endx", type=int,
            help="x-coordinate of the end point of the vehicle"
        )

        parser.add_argument(
            "-ey", "--endy", type=int,
            help="y-coordinate of the end point of the vehicle"
        )

        parser.add_argument(
            "-ox", "--sizex", type=int,
            help="size of objects along the x-axis"
        )

        parser.add_argument(
            "-oy", "--sizey", type=int,
            help="size of objects along the y-axis"
        )

        parser.add_argument(
            "-oc", "--obstaclecount", type=int,
            help="the number of obstacles in the environment"
        )

        parser.add_argument(
            "-ot", "--obstacletheta", type=int,
            help="the orientation of obstacles (in degrees)"
        )

        if algorithm == "genetic":
            parser.add_argument(
                "-pc", "--populationcount", type=int,
                help="the size of the population"
            )

            parser.add_argument(
                "-in", "--interpolation", type=int,
                help="the number of intermediary points along a line segment"
            )

            parser.add_argument(
                "-ps", "--pathsegments", type=int,
                help="the number of segments in a given path"
            )

            parser.add_argument(
                "-cs", "--curvesamples", type=int,
                help="the number of samples to use when path smoothing"
            )

            parser.add_argument(
                "-mc", "--mutationchance", type=float,
                help="the probability that mutation will occur [0.0, 1.0]"
            )

            parser.add_argument(
                "-ev", "--evolutionmax", type=int,
                help="the number of evolutions to carry out"
            )

        args = parser.parse_known_args()[0]

        inputs = Inputs(algorithm)

        inputs.verbose = args.verbose
        inputs.plot = args.plot

        if args.leftx is not None and args.rightx is not None and args.leftx >= args.rightx:
            raise Exception("x-coordinate of the bottom-left point cannot be greater than or equal to the top-right point")

        if args.lefty is not None and args.righty is not None and args.lefty >= args.righty:
            raise Exception("y-coordinate of the bottom-left point cannot be greater than or equal to the top-right point")

        if args.leftx is not None and args.rightx is not None and args.rightx - args.leftx < 8:
            raise Exception("the width of the grid should not be less than 8 units")

        if args.lefty is not None and args.righty is not None and args.righty - args.lefty < 8:
            raise Exception("the height of the grid should not be less than 8 units")

        inputs.grid_minimum = Point(
            args.leftx if args.leftx is not None else (args.rightx - 20.0 if args.rightx is not None else 0.0),
            args.lefty if args.lefty is not None else (args.righty - 20.0 if args.righty is not None else 0.0)
        )

        inputs.grid_maximum = Point(
            args.rightx if args.rightx is not None else (args.leftx + 20.0 if args.leftx is not None else 20.0),
            args.righty if args.righty is not None else (args.lefty + 20.0 if args.lefty is not None else 20.0)
        )

        if args.startx is not None and (args.startx < inputs.grid_minimum.x or args.startx > inputs.grid_minimum.x):
            raise Exception("x-coordinate of the start point of the vehicle must reside within the grid")

        if args.starty is not None and (args.starty < inputs.grid_minimum.y or args.starty > inputs.grid_minimum.y):
            raise Exception("y-coordinate of the start point of the vehicle must reside within the grid")

        if args.endx is not None and (args.endx < inputs.grid_maximum.x or args.endx > inputs.grid_maximum.x):
            raise Exception("x-coordinate of the end point of the vehicle must reside within the grid")

        if args.endy is not None and (args.endy < inputs.grid_maximum.y or args.endy > inputs.grid_maximum.y):
            raise Exception("y-coordinate of the end point of the vehicle must reside within the grid")

        inputs.start = Point(
            args.startx if args.startx is not None else inputs.grid_minimum.x + 2,
            args.starty if args.starty is not None else inputs.grid_minimum.y + 2
        )

        inputs.goal = Point(
            args.endx if args.endx is not None else inputs.grid_maximum.x - 2,
            args.endy if args.endy is not None else inputs.grid_maximum.y - 2
        )

        if args.sizex is not None and args.sizex <= 0.0:
            raise Exception("the size of objects along the x-axis should not be less than or equal to zero")

        if args.sizey is not None and args.sizey <= 0.0:
            raise Exception("the size of objects along the y-axis should not be less than or equal to zero")

        inputs.obstacle_size = Point(
            args.sizex if args.sizex is not None else 1.0,
            args.sizey if args.sizey is not None else 1.0
        )

        inputs.obstacle_count = args.obstaclecount if args.obstaclecount is not None else 40

        inputs.obstacle_theta = args.obstacletheta if args.obstacletheta is not None else 0.0

        if algorithm == "genetic":
            if args.populationcount is not None and args.populationcount < 10:
                raise Exception("the population size should not be less than 10")

            if args.pathsegments is not None and args.pathsegments < 2:
                raise Exception("the value for path segments cannot be less than 2, as that implies a straight line")

            if args.mutationchance is not None and (args.mutationchance < 0.0 or args.mutationchance > 1.0):
                raise Exception("the probability that mutation will occur must be between 0.0 and 1.0 (inclusive)")

            inputs.population_count = args.populationcount if args.populationcount is not None else 80
            inputs.interpolation = args.interpolation if args.interpolation is not None else 8
            inputs.path_segments = args.pathsegments if args.pathsegments is not None else 2
            inputs.curve_samples = args.curvesamples if args.curvesamples is not None else 16
            inputs.mutation_chance = args.mutationchance if args.mutationchance is not None else 0.04
            inputs.evolution_max = args.evolutionmax if args.evolutionmax is not None else 10
        
        return inputs