import argparse

from common.geometry import Point


def parse(filename):
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "-v", "--verbose", action="store_true",
        help="enables additional logging with extra information"
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

    if filename == "genetic-algorithm":
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

    arguments = {
        "verbose": args.verbose
    }

    if args.leftx is not None and args.rightx is not None and args.leftx >= args.rightx:
        raise Exception("x-coordinate of the bottom-left point cannot be greater than or equal to the top-right point")

    if args.lefty is not None and args.righty is not None and args.lefty >= args.righty:
        raise Exception("y-coordinate of the bottom-left point cannot be greater than or equal to the top-right point")

    if args.leftx is not None and args.rightx is not None and args.rightx - args.leftx < 8:
        raise Exception("the width of the grid should not be less than 8 units")

    if args.lefty is not None and args.righty is not None and args.righty - args.lefty < 8:
        raise Exception("the height of the grid should not be less than 8 units")

    arguments["gridMinimum"] = Point(
        args.leftx if args.leftx is not None else (args.rightx - 20.0 if args.rightx is not None else 0.0),
        args.lefty if args.lefty is not None else (args.righty - 20.0 if args.righty is not None else 0.0)
    )

    arguments["gridMaximum"] = Point(
        args.rightx if args.rightx is not None else (args.leftx + 20.0 if args.leftx is not None else 20.0),
        args.righty if args.righty is not None else (args.lefty + 20.0 if args.lefty is not None else 20.0)
    )

    if args.startx is not None and (args.startx < arguments["gridMinimum"].x or args.startx > arguments["gridMaximum"].x):
        raise Exception("x-coordinate of the start point of the vehicle must reside within the grid")

    if args.starty is not None and (args.starty < arguments["gridMinimum"].y or args.starty > arguments["gridMaximum"].y):
        raise Exception("y-coordinate of the start point of the vehicle must reside within the grid")

    if args.endx is not None and (args.endx < arguments["gridMinimum"].x or args.endx > arguments["gridMaximum"].x):
        raise Exception("x-coordinate of the end point of the vehicle must reside within the grid")

    if args.endy is not None and (args.endy < arguments["gridMinimum"].y or args.endy > arguments["gridMaximum"].y):
        raise Exception("y-coordinate of the end point of the vehicle must reside within the grid")

    arguments["vehicleFirst"] = Point(
        args.startx if args.startx is not None else arguments["gridMinimum"].x + 2,
        args.starty if args.starty is not None else arguments["gridMinimum"].y + 2
    )

    arguments["vehicleFinal"] = Point(
        args.endx if args.endx is not None else arguments["gridMaximum"].x - 2,
        args.endy if args.endy is not None else arguments["gridMaximum"].y - 2
    )

    if args.sizex is not None and args.sizex <= 0.0:
        raise Exception("the size of objects along the x-axis should not be less than or equal to zero")

    if args.sizey is not None and args.sizey <= 0.0:
        raise Exception("the size of objects along the y-axis should not be less than or equal to zero")

    arguments["objectSize"] = Point(
        args.sizex if args.sizex is not None else 1.0,
        args.sizey if args.sizey is not None else 1.0
    )

    arguments["obstacleCount"] = args.obstaclecount if args.obstaclecount is not None else 40

    arguments["obstacleTheta"] = args.obstacletheta if args.obstacletheta is not None else 0.0

    if filename == "genetic-algorithm":
        if args.populationcount is not None and args.populationcount < 10:
            raise Exception("the population size should not be less than 10")

        if args.pathsegments is not None and args.pathsegments < 2:
            raise Exception("the value for path segments cannot be less than 2, as that implies a straight line")

        if args.mutationchance is not None and (args.mutationchance < 0.0 or args.mutationchance > 1.0):
            raise Exception("the probability that mutation will occur must be between 0.0 and 1.0 (inclusive)")

        arguments["populationCount"] = args.populationcount if args.populationcount is not None else 80
        arguments["interpolation"] = args.interpolation if args.interpolation is not None else 8
        arguments["pathSegments"] = args.pathsegments if args.pathsegments is not None else 2
        arguments["curveSamples"] = args.curvesamples if args.curvesamples is not None else 16
        arguments["mutationChance"] = args.mutationchance if args.mutationchance is not None else 0.04
        arguments["evolutionMax"] = args.evolutionmax if args.evolutionmax is not None else 10

    return arguments
