import numpy as np

# Lightweight Catmull-Rom spline sampling (no scipy dependency)
# We generate a dense polyline representing the smoothed curve.

def catmull_rom_chain(pts, samples_per_segment=50, closed=False):
    """Compute a Catmull-Rom spline through pts and return sampled points.

    pts: (N,2) array
    returns: (M,2) array of sampled points
    """
    pts = np.asarray(pts, dtype=float)
    if pts.shape[0] < 2:
        return pts.copy()

    # For endpoints, create virtual control points by repeating end points
    if closed:
        extended = np.vstack([pts[-2:], pts, pts[:2]])
    else:
        # pad start and end with endpoint duplicates
        extended = np.vstack([pts[0], pts, pts[-1]])

    out = []
    n_segments = extended.shape[0] - 3
    for i in range(n_segments):
        p0 = extended[i]
        p1 = extended[i + 1]
        p2 = extended[i + 2]
        p3 = extended[i + 3]
        for j in range(samples_per_segment):
            t = j / float(samples_per_segment)
            t2 = t * t
            t3 = t2 * t
            # Catmull-Rom basis
            f1 = -0.5 * t3 + t2 - 0.5 * t
            f2 = 1.5 * t3 - 2.5 * t2 + 1.0
            f3 = -1.5 * t3 + 2.0 * t2 + 0.5 * t
            f4 = 0.5 * t3 - 0.5 * t2
            point = p0 * f1 + p1 * f2 + p2 * f3 + p3 * f4
            out.append(point)
    out.append(pts[-1].copy())
    return np.array(out)


def smooth_path(waypoints, samples_per_segment=50):
    """Public smoothing function used by demo.
    Returns a dense Nx2 array of points along the smooth curve.
    """
    return catmull_rom_chain(waypoints, samples_per_segment=samples_per_segment)
