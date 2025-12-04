import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

import numpy as np
from smoothing import smooth_path


def test_smoothing_basic():
    pts = np.array([[0, 0], [1, 0], [2, 0]])
    s = smooth_path(pts, samples_per_segment=10)
    # smoothing of a straight line should keep y nearly zero
    assert s.shape[0] >= 2
    assert abs(s[:, 1]).max() < 1e-6
