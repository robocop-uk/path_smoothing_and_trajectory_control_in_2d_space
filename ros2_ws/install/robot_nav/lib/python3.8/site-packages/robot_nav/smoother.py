import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

def smooth_path(waypoints, num_points=100):
    waypoints = np.array(waypoints)
    x = waypoints[:, 0]
    y = waypoints[:, 1]
    dist = np.cumsum(np.sqrt(np.diff(x, prepend=x[0])**2 + np.diff(y, prepend=y[0])**2))
    dist[0] = 0
    cs_x = CubicSpline(dist, x)
    cs_y = CubicSpline(dist, y)
    dist_fine = np.linspace(dist[0], dist[-1], num_points)
    x_smooth = cs_x(dist_fine)
    y_smooth = cs_y(dist_fine)
    return list(zip(x_smooth, y_smooth))

if __name__ == "__main__":
    waypoints = [(0, 0), (1, 2), (3, 3), (5, 2), (6, 0)]
    path = smooth_path(waypoints, num_points=200)
    x_orig, y_orig = zip(*waypoints)
    x_smooth, y_smooth = zip(*path)
    plt.plot(x_orig, y_orig, 'ro--', label='Original')
    plt.plot(x_smooth, y_smooth, 'b-', label='Smoothed')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()
