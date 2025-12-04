import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

import numpy as np
from trajectory import generate_trajectory


def test_trajectory_monotonic_times():
    poly = np.array([[0, 0], [1, 0], [2, 0]])
    traj = generate_trajectory(poly, n_samples=50, v_max=1.0, a_max=0.5)
    times = traj['times']
    assert all(np.diff(times) >= 0)
