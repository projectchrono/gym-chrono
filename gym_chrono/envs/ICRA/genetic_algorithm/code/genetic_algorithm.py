import time

import numpy as np

import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__)))

import common.environment as environment
import common.inputs as inputs

from common.visualize import visualizeResult, scatterPlot
from common.geometry import Point, Line
from common.smooth import bezierCurve

class Path:
    def __init__(self, points):
        self.score = np.inf
        self.points = points

    def fitness(self, obstacles, shortest):
        distance = 0
        collisions = 0

        for i in range(len(self.points) - 1):
            segment = Line(self.points[i], self.points[i + 1])
            distance += segment.length
            for obstacle in obstacles:
                if segment.intersects(obstacle):
                    collisions += 1000

        self.score = np.sqrt((distance / shortest.length) ** 2 + collisions ** 2)


def individual(grid, interpolation, segments):
    points = [grid.first]

    for s in range(segments):
        nextPoint = grid.random(grid.size, Point(0, 0)) if s < segments-1 else grid.final
        segment = Line(points[-1], nextPoint)
        points.extend(segment.divide(interpolation))

    return points


def evolve(population, grid, count, chance):
    children = []
    while len(children) < count:
        parentA = np.random.randint(0, len(population))
        parentB = np.random.randint(0, len(population))
        if parentA != parentB:
            pathA = population[parentA].points
            pathB = population[parentB].points
            crossoverPosition = len(pathA) // 2
            child = pathA[:crossoverPosition] + pathB[crossoverPosition:]
            if np.random.random() <= chance:
                mutationPosition = np.random.randint(0, len(child))
                child[mutationPosition] = grid.random(grid.size, Point(0, 0))
            children.append(child)
    return children


def select(graded, evolved, count):
    graded.extend(evolved)
    graded = sort(graded)

    # truncation selection
    if len(graded) > count:
        graded = graded[:count]

    return graded


def sort(population):
    if len(population) <= 1:
        return population

    mid = len(population) // 2

    left = sort(population[:mid])
    right = sort(population[mid:])

    return merge(left, right, population.copy())


def merge(left, right, population):
    leftPosition = 0
    rightPosition = 0

    while leftPosition < len(left) and rightPosition < len(right):

        if left[leftPosition].score <= right[rightPosition].score:
            population[leftPosition + rightPosition] = left[leftPosition]
            leftPosition += 1
        else:
            population[leftPosition + rightPosition] = right[rightPosition]
            rightPosition += 1

    for leftPosition in range(leftPosition, len(left)):
        population[leftPosition + rightPosition] = left[leftPosition]

    for rightPosition in range(rightPosition, len(right)):
        population[leftPosition + rightPosition] = right[rightPosition]

    return population

def create_path(obstacles=None):
    filename = "genetic-algorithm"
    arguments = inputs.parse(filename)

    arguments["gridMinimum"] = Point(-40,-40)
    arguments["gridMaximum"] = Point(40,40)
    arguments["vehicleFirst"] = Point(-35,-35)
    arguments["vehicleFinal"] = Point(35,35)
    arguments["objectSize"] = Point(6,6)
    arguments["obstacleCount"] = 100

    grid, boundaries, obstacles = environment.generate(arguments, obstacles)

    startTime = time.time()

    shortestPath = Line(grid.first, grid.final)

    initialPopulation = []

    for _ in range(arguments["populationCount"]):
        path = Path(individual(grid, arguments["interpolation"], arguments["pathSegments"]))
        path.points = bezierCurve(path.points, arguments["curveSamples"])
        path.fitness(obstacles, shortestPath)
        initialPopulation.append(path)

    gradedPopulation = sort(initialPopulation)

    finalPopulation = None
    optimalPath = None

    averageFitness = []
    evolutionCount = 0

    while evolutionCount < arguments["evolutionMax"]:
        evolvedPaths = evolve(gradedPopulation, grid, arguments["populationCount"], arguments["mutationChance"])

        evolvedPopulation = []

        for points in evolvedPaths:
            path = Path(points)
            path.fitness(obstacles, shortestPath)
            evolvedPopulation.append(path)

        gradedPopulation = select(gradedPopulation, evolvedPopulation, arguments["populationCount"])

        average = 0
        for path in gradedPopulation:
            average += path.score
        average /= len(gradedPopulation)

        averageFitness.append(average)

        if arguments["verbose"]:
            print(
                "Evolution:", evolutionCount + 1,
                "| Average Fitness:", average,
                "| Best Fitness Value:", gradedPopulation[0].score
            )

        evolutionCount += 1

        if evolutionCount == arguments["evolutionMax"]:
            finalPopulation = gradedPopulation
            optimalPath = gradedPopulation[0]

    endTime = time.time()

    print("Time Elapsed:", endTime - startTime)

    visualizeResult(grid, boundaries, obstacles, "Optimal Path", None, optimalPath)

    return optimalPath

def main():
    filename = "genetic-algorithm"
    arguments = inputs.parse(filename)

    arguments["gridMinimum"] = Point(-50,-50)
    arguments["gridMaximum"] = Point(50,50)
    arguments["vehicleFirst"] = Point(-35,-35)
    arguments["vehicleFinal"] = Point(35,35)
    arguments["obstacleSize"] = Point(5,5)
    arguments["obstacleCount"] = 100

    grid, boundaries, obstacles = environment.generate(arguments)
    visualizeResult(grid, boundaries, obstacles, "Environment")

    startTime = time.time()

    shortestPath = Line(grid.first, grid.final)

    initialPopulation = []

    for _ in range(arguments["populationCount"]):
        path = Path(individual(grid, arguments["interpolation"], arguments["pathSegments"]))
        path.points = bezierCurve(path.points, arguments["curveSamples"])
        path.fitness(obstacles, shortestPath)
        initialPopulation.append(path)

    gradedPopulation = sort(initialPopulation)

    finalPopulation = None
    optimalPath = None

    averageFitness = []
    evolutionCount = 0

    while evolutionCount < arguments["evolutionMax"]:
        evolvedPaths = evolve(gradedPopulation, grid, arguments["populationCount"], arguments["mutationChance"])

        evolvedPopulation = []

        for points in evolvedPaths:
            path = Path(points)
            path.fitness(obstacles, shortestPath)
            evolvedPopulation.append(path)

        gradedPopulation = select(gradedPopulation, evolvedPopulation, arguments["populationCount"])

        average = 0
        for path in gradedPopulation:
            average += path.score
        average /= len(gradedPopulation)

        averageFitness.append(average)

        if arguments["verbose"]:
            print(
                "Evolution:", evolutionCount + 1,
                "| Average Fitness:", average,
                "| Best Fitness Value:", gradedPopulation[0].score
            )

        evolutionCount += 1

        if evolutionCount == arguments["evolutionMax"]:
            finalPopulation = gradedPopulation
            optimalPath = gradedPopulation[0]

    endTime = time.time()

    print("Time Elapsed:", endTime - startTime)

    # visualizeResult(grid, boundaries, obstacles, "Initial Population", initialPopulation)
    # visualizeResult(grid, boundaries, obstacles, "Final Population", finalPopulation)
    visualizeResult(grid, boundaries, obstacles, "Optimal Path", None, optimalPath)

    # scatterPlot(
    #     np.arange(1, arguments["evolutionMax"] + 1), averageFitness,
    #     "Average Fitness of Population", "Evolution", "Fitness Value"
    # )


if __name__ == "__main__":
    main()
