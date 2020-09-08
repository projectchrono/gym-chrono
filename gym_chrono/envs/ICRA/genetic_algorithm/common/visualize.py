import matplotlib.pyplot as plt
import matplotlib.patches as ptc
from descartes import PolygonPatch

from common.geometry import Line

def closeFigs():
    plt.close('all')

def visualizeResult(grid, boundaries, obstacles, title, population=None, optimal=None):
    fig, ax = plt.subplots()

    ax.set_title(title, weight='bold')

    ax.annotate(
        "FIRST", (grid.first.x, grid.minimum.y - 0.1),
        horizontalalignment='center',
        verticalalignment='top',
        weight='bold', color='w',
    )

    ax.annotate(
        "FINAL", (grid.final.x, grid.maximum.y + 0.0),
        horizontalalignment='center',
        verticalalignment='bottom',
        weight='bold', color='w'
    )

    for obstacle in obstacles:
        patch = PolygonPatch(obstacle, edgecolor='None', facecolor='grey', alpha=1.0)
        ax.add_patch(patch)

    for boundary in boundaries:
        rectangle = ptc.Rectangle(
            boundary.datum,
            boundary.width,
            boundary.height,
            boundary.angle(),
            edgecolor='None', facecolor='black', alpha=1.0
        )
        ax.add_patch(rectangle)

    if population is not None:
        for path in population:
            px = [point.x for point in path.points]
            py = [point.y for point in path.points]
            ax.plot(px, py, 'y-', alpha=0.2, markersize=4)

    if optimal is not None:
        px = [point.x for point in optimal.points]
        py = [point.y for point in optimal.points]
        ax.plot(px, py, '.c-', alpha=0.8, markersize=4)

        ls = Line(points=optimal.points).buffer(2.0)
        patch = PolygonPatch(ls, edgecolor='None', facecolor='grey', alpha=0.3)
        ax.add_patch(patch)

    ax.plot(grid.first.x, grid.first.y, 'co')
    ax.plot(grid.final.x, grid.final.y, 'mo')

    ax.grid()

    plt.axis('scaled')

    plt.ion()
    plt.show()


def scatterPlot(x, y, title, xlabel, ylabel):
    fig, ax = plt.subplots()
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    plt.scatter(x, y, marker='o', c='c')
    plt.grid()
    plt.ion()
    plt.show()