# smoother.py

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

def smooth_path(waypoints, num_points=100):
    """
    Smooths a given list of waypoints using cubic splines.

    Args:
        waypoints (list of tuple): List of (x, y) waypoints.
        num_points (int): Number of interpolated points in the smooth path.

    Returns:
        smooth_path (list of tuple): Smooth interpolated (x, y) path.
    """

    waypoints = np.array(waypoints)
    x = waypoints[:, 0]
    y = waypoints[:, 1]

    # Use the cumulative distance along the path as a parameter
    dist = np.cumsum(np.sqrt(np.diff(x, prepend=x[0])**2 + np.diff(y, prepend=y[0])**2))
    dist[0] = 0  # ensure first value is 0

    # Fit splines
    cs_x = CubicSpline(dist, x)
    cs_y = CubicSpline(dist, y)

    # Interpolate using fine-grained distance samples
    dist_fine = np.linspace(dist[0], dist[-1], num_points)
    x_smooth = cs_x(dist_fine)
    y_smooth = cs_y(dist_fine)

    smooth_path = list(zip(x_smooth, y_smooth))
    return smooth_path


# Optional: Test the function with a sample plot
if __name__ == "__main__":
    waypoints = [(0, 0), (1, 2), (3, 3), (5, 2), (6, 0)]
    path = smooth_path(waypoints, num_points=200)

    x_orig, y_orig = zip(*waypoints)
    x_smooth, y_smooth = zip(*path)

    plt.figure(figsize=(8, 6))
    plt.plot(x_orig, y_orig, 'ro--', label='Original Waypoints')
    plt.plot(x_smooth, y_smooth, 'b-', label='Smoothed Path')
    plt.legend()
    plt.title('Path Smoothing with Cubic Splines')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.axis('equal')
    plt.show()

