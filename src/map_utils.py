import numpy as np

def example_waypoints():
    """Return a simple list of waypoints useful for demos.
    Format: list of (x,y) tuples.
    """
    pts = [(0.0, 0.0), (2.0, 0.0), (4.0, 2.0), (6.0, 0.0), (8.0, 0.0)]
    return np.array(pts)


def load_waypoints_from_csv(path):
    """Load waypoints from a CSV file with two columns x,y"""
    data = np.loadtxt(path, delimiter=',')
    if data.ndim == 1:
        data = data.reshape((1, -1))
    return data[:, :2]
