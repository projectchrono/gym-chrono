import numpy as np

from scipy.special import comb
from common.geometry import Point


def bezierCurve(points, samples):
    t = np.linspace(0.0, 1.0, samples)

    px = [p.x for p in points]
    py = [p.y for p in points]

    polynomials = [bernsteinPolynomial(len(points) - 1, i, t) for i in range(len(points))]

    vx = np.dot(px, polynomials)
    vy = np.dot(py, polynomials)

    return [Point(vx[s], vy[s]) for s in range(samples)]


def bernsteinPolynomial(n, i, t):
    return comb(n, i) * (t ** i) * ((1 - t) ** (n - i))
