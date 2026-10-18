"""Density ridges of a horizontal slice of a point cloud: the line through the densest part of each wall.

Used by plan_section.py and wall_precision.py. The slice points are counted in a cell grid, the counts smoothed
(Gaussian, sigma cells), and a cell is a ridge cell when its density is above the neighbour on one side and at least that
on the other (one cell wide) along the direction of strongest downward curvature (the Hessian's most negative
eigenvector), the curvature there is negative, and the density is at least threshold times the 99th percentile. Pieces
whose extent (bounding-box diagonal) is under min_length are dropped.
"""
import numpy as np
from scipy import ndimage


class Section:
    def __init__(self, x, y, cell=0.02, sigma=3.0):
        self.cell = cell
        self.x0, self.y0 = x.min() - 1.0, y.min() - 1.0
        nx, ny = int((x.max() + 1.0 - self.x0) / cell) + 1, int((y.max() + 1.0 - self.y0) / cell) + 1
        G, _, _ = np.histogram2d(x, y, bins=(nx, ny), range=((self.x0, self.x0 + nx * cell), (self.y0, self.y0 + ny * cell)))
        D = ndimage.gaussian_filter(G, sigma)
        dxx = ndimage.gaussian_filter(G, sigma, order=(2, 0))
        dyy = ndimage.gaussian_filter(G, sigma, order=(0, 2))
        dxy = ndimage.gaussian_filter(G, sigma, order=(1, 1))
        lam = 0.5 * (dxx + dyy) - np.sqrt(0.25 * (dxx - dyy) ** 2 + dxy ** 2)  # most negative curvature
        vx, vy = dxy, lam - dxx                                                 # its eigenvector
        alt = np.abs(vx) + np.abs(vy) < 1e-12
        vx, vy = np.where(alt, lam - dyy, vx), np.where(alt, dxy, vy)
        nrm = np.hypot(vx, vy) + 1e-30
        self.nx_, self.ny_ = vx / nrm, vy / nrm                                 # across the wall
        ii, jj = np.mgrid[0:nx, 0:ny].astype(float)
        d_plus = ndimage.map_coordinates(D, [ii + self.nx_, jj + self.ny_], order=1, mode="nearest")
        d_minus = ndimage.map_coordinates(D, [ii - self.nx_, jj - self.ny_], order=1, mode="nearest")
        self.D = D
        self.peak = np.percentile(D[D > 0], 99)
        self.candidate = (D > d_plus) & (D >= d_minus) & (lam < 0)

    def ridge_cells(self, threshold, min_length=0.5):
        """ridge mask for one threshold: density at least threshold * peak, pieces spanning at least min_length"""
        ridge = self.candidate & (self.D >= threshold * self.peak)
        lab, n = ndimage.label(ridge, structure=np.ones((3, 3)))
        boxes = ndimage.find_objects(lab)                  # a piece's extent, so a curled blob doesn't count as long
        size = np.array([np.hypot(b[0].stop - b[0].start, b[1].stop - b[1].start) * self.cell for b in boxes])
        return np.isin(lab, 1 + np.flatnonzero(size >= min_length))

    def centres(self, ri, rj):
        """map xy of grid cells"""
        return np.column_stack([self.x0 + (ri + 0.5) * self.cell, self.y0 + (rj + 0.5) * self.cell])
