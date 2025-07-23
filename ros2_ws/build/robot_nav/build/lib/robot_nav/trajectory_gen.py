import numpy as np

def generate_trajectory(smooth_path, velocity=0.2):
    if len(smooth_path) < 2:
        raise ValueError("Need at least 2 points")
    trajectory = []
    t = 0.0
    trajectory.append((*smooth_path[0], t))
    for i in range(1, len(smooth_path)):
        x0, y0 = smooth_path[i - 1]
        x1, y1 = smooth_path[i]
        dist = np.linalg.norm([x1 - x0, y1 - y0])
        t += dist / velocity
        trajectory.append((x1, y1, t))
    return trajectory

if __name__ == "__main__":
    from smoother import smooth_path
    waypoints = [(0, 0), (1, 2), (3, 3), (5, 2), (6, 0)]
    smooth = smooth_path(waypoints)
    traj = generate_trajectory(smooth)
    print(traj[:5])
