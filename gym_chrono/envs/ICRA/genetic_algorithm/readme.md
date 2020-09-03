## Path Planning

The implementation of an optimal path planning algorithm for autonomous vehicles is crucial to their ability to successfully traverse through a static or a dynamic environment. Traditional analytical approaches to path planning are often dependent upon the existence of derivatives, continuity, and unimodality in order to find a globally optimal solution. Classical enumeration methods come close to finding optimal solutions, but they become inefficient as the dimensionality of the problem increases. This project examines the applicability of genetic algorithms, as a search and optimization heuristic technique, for the purpose of path planning.

## Algorithms

Though the focus of this project is on genetic algorithms, it is nonetheless valuable to compare the strengths and weaknesses of several different path planning algorithms. The list below outlines the algorithms for which code has been developed. There are quite a few variations that exist for each algorithm and command-line arguments can be used to customize the behavior. Please refer to the next section below for more details.

- Genetic Algorithms
- Dijkstra's Algorithm
- A-Star (A*) Algorithms
- D-Star (D*) Algorithms
- Artificial Potential Fields
- Rapidly Exploring Random Trees

## Input Arguments

The command-line arguments that are independent of the algorithm being executed are listed below:

| Short Option | Long Option | Description |
|:-------------|:------------|:------------|
|`-h`|`--help`|show the help message and exit|
|`-v`|`--verbose`|enables additional logging with extra information|
|`-lx`|`--leftx`|x-coordinate of the bottom-left point of the grid|
|`-ly`|`--lefty`|y-coordinate of the bottom-left point of the grid|
|`-rx`|`--rightx`|x-coordinate of the top-right point of the grid|
|`-ry`|`--righty`|y-coordinate of the top-right point of the grid|
|`-sx`|`--startx`|x-coordinate of the start point of the vehicle|
|`-sy`|`--starty`|y-coordinate of the start point of the vehicle|
|`-ex`|`--endx`|x-coordinate of the end point of the vehicle|
|`-ey`|`--endy`|y-coordinate of the end point of the vehicle|
|`-ox`|`--sizex`|size of objects along the x-axis|
|`-oy`|`--sizey`|size of objects along the y-axis|
|`-oc`|`--obstaclecount`|the number of obstacles in the environment|
|`-ot`|`--obstacletheta`|the orientation of obstacles (in degrees)|

The command-line arguments that are specific to genetic algorithms are listed below:

| Short Option | Long Option | Description |
|:-------------|:------------|:------------|
|`-pc`|`--populationcount`|the size of the population|
|`-in`|`--interpolation`|the number of intermediary points along a line segment|
|`-ps`|`--pathsegments`|the number of segments in a given path|
|`-cs`|`--curvesamples`|the number of samples to use when path smoothing|
|`-mc`|`--mutationchance`|the probability that mutation will occur [0.0, 1.0]|
|`-ev`|`--evolutionmax`|the number of evolutions to carry out|

## Installation

It is recommended that you create a new virtual environment:

```
conda create -n path-planning python=3.6
```

The environment must first be activated:

```
conda activate path-planning
```

The following dependencies need to be installed:

```
conda install numpy matplotlib scipy shapely
```

The code can then be executed, for example via the command-line:

```
python genetic-algorithm.py
```

## Visualization

The results from running the genetic algorithm code (with the default arguments) are shown below:

| Initial Population | Convergence | Optimal Path | Average Fitness |
|:------------------:|:-----------:|:------------:|:---------------:|
|    ![I][start]     |![C][middle] |  ![O][stop]  |  ![A][average]  |

[start]: images/start.png "Initial Population"
[middle]: images/middle.png "Convergence"
[stop]: images/stop.png "Optimal Path"
[average]: images/average.png "Average Fitness"
