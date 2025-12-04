import numpy as np


def arc_length_sample(polyline, n_samples):
    """Given polyline (M,2), sample n_samples points evenly spaced by distance.
    Returns (n_samples,2) points and distances along the path.
    """
    pts = np.asarray(polyline)
    seg = np.diff(pts, axis=0)
    seg_len = np.hypot(seg[:, 0], seg[:, 1])
    cum = np.concatenate([[0.0], np.cumsum(seg_len)])
    total = cum[-1]
    if total == 0 or n_samples == 1:
        return pts[[0]], np.array([0.0])
    distances = np.linspace(0.0, total, n_samples)
    xs = np.interp(distances, cum, pts[:, 0])
    ys = np.interp(distances, cum, pts[:, 1])
    return np.vstack([xs, ys]).T, distances


def trapezoidal_time_profile(total_dist, v_max=1.0, a_max=0.5):
    """Compute time at each distance along a path using a trapezoidal profile.
    Returns a function t(s) that maps distance s (0..total_dist) to time.
    We will compute times for a vector of distances.
    """
    # time to accelerate to v_max
    t_acc = v_max / a_max
    d_acc = 0.5 * a_max * t_acc * t_acc
    if 2 * d_acc >= total_dist:
        # triangular profile (never reaches v_max)
        # peak velocity v_peak: v_peak^2 = 2 * a * total_dist / 1
        v_peak = (2 * a_max * total_dist) ** 0.5
        t_peak = v_peak / a_max
        def time_of_s(s):
            # accelerate until s <= d_acc_peak
            d_half = total_dist / 2.0
            if np.isscalar(s):
                s = np.array([s])
            s = np.asarray(s)
            t = np.empty_like(s)
            # for s <= d_half: t = sqrt(2*s/a)
            mask = s <= d_half
            t[mask] = (2.0 * s[mask] / a_max) ** 0.5
            # for s > d_half: decel phase
            t[~mask] = t_peak + (2 * (s[~mask] - d_half) / a_max) ** 0.5
            return t
        return time_of_s
    else:
        d_cruise = total_dist - 2 * d_acc
        t_cruise = d_cruise / v_max
        def time_of_s(s):
            if np.isscalar(s):
                s = np.array([s])
            s = np.asarray(s)
            t = np.empty_like(s)
            # accel region
            mask1 = s <= d_acc
            t[mask1] = (2.0 * s[mask1] / a_max) ** 0.5
            # cruise region
            mask2 = (s > d_acc) & (s <= d_acc + d_cruise)
            t[mask2] = t_acc + (s[mask2] - d_acc) / v_max
            # decel region
            mask3 = s > d_acc + d_cruise
            t[mask3] = t_acc + t_cruise + (2 * (s[mask3] - d_acc - d_cruise) / a_max) ** 0.5
            return t
        return time_of_s


def generate_trajectory(polyline, n_samples=200, v_max=1.0, a_max=0.5):
    """Given a dense polyline (Mx2), produce a time-parameterized trajectory.
    Returns dict with keys: points (Nx2), distances (N), times (N), velocities (N)
    """
    pts, distances = arc_length_sample(polyline, n_samples)
    total = distances[-1]
    time_of_s = trapezoidal_time_profile(total, v_max=v_max, a_max=a_max)
    times = time_of_s(distances)
    # velocities approximate from derivative of s(t) -> use finite diff
    # v at sample i approx = (s[i+1]-s[i-1])/(t[i+1]-t[i-1])
    v = np.gradient(distances, times)
    return {"points": pts, "distances": distances, "times": times, "v": v}
