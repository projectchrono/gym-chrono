import time

import numpy as np

import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__)))

import common.environment as environment
import common.inputs as inputs

from common.visualize import visualizeResult, scatterPlot, closeFigs
from common.geometry import Point, Line, Polygon
from common.smooth import bezier_curve

class Path:
    def __init__(self, points):
        self.score = np.inf
        self.points = points

    def fitness(self, obstacles, shortest_length):
        ls = Line(points=self.points).buffer(2.0)
        overlap = obstacles.intersection(ls).area

        self.score = np.sqrt((ls.length / shortest_length) ** 2 + overlap ** 2)


def individual(grid, interpolation, segments):
    points = [grid.first]

    for s in range(segments):
        next_point = grid.random_point() if s < segments-1 else grid.final
        segment = Line(points[-1], next_point)
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
            crossover_position = len(pathA) // 2
            child = pathA[:crossover_position] + pathB[crossover_position:]
            if np.random.random() <= chance:
                mutation_position = np.random.randint(0, len(child))
                child[mutation_position] = grid.mutate_point(child[mutation_position])
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
    left_position = 0
    right_position = 0

    while left_position < len(left) and right_position < len(right):

        if left[left_position].score <= right[right_position].score:
            population[left_position + right_position] = left[left_position]
            left_position += 1
        else:
            population[left_position + right_position] = right[right_position]
            right_position += 1

    for left_position in range(left_position, len(left)):
        population[left_position + right_position] = left[left_position]

    for right_position in range(right_position, len(right)):
        population[left_position + right_position] = right[right_position]

    return population

def genetic_algorithm(inputs):
    grid, boundaries, obstacles = environment.generate(inputs)

    start_time = time.time()

    shortest_path = Line(grid.first, grid.final)
    shortest_length = shortest_path.length

    initial_population = []
    for _ in range(inputs.population_count):
        points = individual(grid, inputs.interpolation, inputs.path_segments)
        path = Path(bezier_curve(points, inputs.curve_samples))
        path.fitness(obstacles, shortest_length)
        initial_population.append(path)
    
    graded_population = sort(initial_population)

    final_population = None
    optimal_path = None

    average_fitness = []
    evolution_count = 0

    while evolution_count < inputs.evolution_max:
        evolved_paths = evolve(graded_population, grid, inputs.population_count, inputs.mutation_chance)

        evolved_population = []

        for points in evolved_paths:
            path = Path(bezier_curve(points, inputs.curve_samples))
            path.fitness(obstacles, shortest_length)
            evolved_population.append(path)

        graded_population = select(graded_population, evolved_population, inputs.population_count)

        average = 0
        for path in graded_population:
            average += path.score
        average /= len(graded_population)

        average_fitness.append(average)

        if inputs.verbose:
            print(
                "Evolution:", evolution_count + 1,
                "| Average Fitness:", average,
                "| Best Fitness Value:", graded_population[0].score
            )

        evolution_count += 1

        if evolution_count == inputs.evolution_max:
            final_population = graded_population
            optimal_path = graded_population[0]

    end_time = time.time()

    print("Time Elapsed:", end_time - start_time)

    if inputs.plot:
        visualizeResult(grid, boundaries, obstacles, "Initial Population", population=initial_population)
        visualizeResult(grid, boundaries, obstacles, "Final Population", population=final_population)
        visualizeResult(grid, boundaries, obstacles, "Optimal Path", optimal=optimal_path)

        input("Press Enter to Exit")

        closeFigs()

    return optimal_path

if __name__ == '__main__':
    inputs = inputs.Inputs('genetic')

    inputs.grid_minimum = Point(-50,-50)
    inputs.grid_maximum = Point(50,50)
    inputs.curve_samples = 30
    inputs.population_count = 50
    inputs.start = Point(-35,-35)
    inputs.goal = Point(35,35)
    inputs.plot = True

    # inputs = inputs.Inputs.parse('genetic')

    genetic_algorithm(inputs)