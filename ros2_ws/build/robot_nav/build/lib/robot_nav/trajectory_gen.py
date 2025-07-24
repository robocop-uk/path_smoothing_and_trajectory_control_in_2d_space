# trajectory_gen.py

import numpy as np

def generate_trajectory(smooth_path, velocity=0.2):
    """
    Generate a time-stamped trajectory from a smooth path.

    Args:
        smooth_path (list of tuple): Smoothed path [(x0, y0), (x1, y1), ...]
        velocity (float): Constant linear velocity in m/s

    Returns:
        trajectory (list of tuple): [(x, y, t), ...] time-stamped trajectory
    """

    if len(smooth_path) < 2:
        raise ValueError("Need at least 2 points to generate trajectory")

    trajectory = []
    t = 0.0
    trajectory.append((*smooth_path[0], t))

    for i in range(1, len(smooth_path)):
        x0, y0 = smooth_path[i - 1]
        x1, y1 = smooth_path[i]

        dist = np.linalg.norm([x1 - x0, y1 - y0])
        dt = dist / velocity
        t += dt

        trajectory.append((x1, y1, t))

    return trajectory


# Optional: Test this function directly
if __name__ == "__main__":
    from smoother import smooth_path

    waypoints = [(0, 0), (1, 2), (3, 3), (5, 2), (6, 0)]
    smooth = smooth_path(waypoints, num_points=100)
    trajectory = generate_trajectory(smooth, velocity=0.2)

    print("First 5 trajectory points:")
    for point in trajectory[:5]:
        print(f"x: {point[0]:.2f}, y: {point[1]:.2f}, t: {point[2]:.2f}")

