"""Demo script: generate smoothed path, time-parameterize it, and run simulator."""
import os
import sys

# Add src folder to path so this works when run via `python src/demo.py`
_src_dir = os.path.dirname(__file__)
if _src_dir not in sys.path:
    sys.path.insert(0, _src_dir)

import numpy as np

from map_utils import example_waypoints
from smoothing import smooth_path
from trajectory import generate_trajectory
from simulator import Simulator


def main():
    waypoints = example_waypoints()
    print("Waypoints:")
    print(waypoints)

    smooth = smooth_path(waypoints, samples_per_segment=80)
    traj = generate_trajectory(smooth, n_samples=300, v_max=1.0, a_max=0.7)

    sim = Simulator(traj, dt=0.05, v_des=0.8)
    history = sim.run(x0=(waypoints[0, 0], waypoints[0, 1], 0.0), show=True)

    out_dir = os.path.join(os.getcwd(), 'out')
    os.makedirs(out_dir, exist_ok=True)
    np.savetxt(os.path.join(out_dir, 'history.csv'), history, delimiter=',')
    print('Simulation finished. History saved to out/history.csv')


if __name__ == '__main__':
    main()
