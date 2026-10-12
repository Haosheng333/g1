"""Pure numpy/scipy helpers shared by slam_manager, relocalizer (and tests).

No rospy import here on purpose, so everything in this file can be unit-tested
without a ROS master.

Frame conventions (see slam/README.md section 3 and FIX_GUIDE.md):
  * camera_init : FAST-LIO2 odometry origin. Z points INTO the floor (the
                  Mid-360 is mounted upside down).  Saved PCDs live here.
  * map         : z-UP navigation frame.  map -> camera_init (of the mapping
                  session) is a pure roll of pi, so  p_map = (x, -y, -z).
"""
import math

import numpy as np

# 21 bits per axis packed into one int64 voxel key.
_OFFSET = 1 << 20


# --------------------------------------------------------------------------
# PointCloud2 -> numpy
# --------------------------------------------------------------------------
def pc2_to_xyz(msg):
    """sensor_msgs/PointCloud2 -> (N, 3) float32, NaN/inf points removed."""
    offsets = {}
    for f in msg.fields:
        if f.name in ("x", "y", "z"):
            if f.datatype != 7:  # PointField.FLOAT32
                raise ValueError("field %s is not FLOAT32" % f.name)
            offsets[f.name] = f.offset
    if len(offsets) != 3:
        raise ValueError("PointCloud2 has no x/y/z fields")
    n = int(msg.width) * int(msg.height)
    if n == 0:
        return np.zeros((0, 3), dtype=np.float32)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    raw = raw[: n * msg.point_step].reshape(n, msg.point_step)
    cols = []
    for axis in ("x", "y", "z"):
        o = offsets[axis]
        col = np.ascontiguousarray(raw[:, o:o + 4]).view("<f4").reshape(n)
        cols.append(col)
    pts = np.stack(cols, axis=1).astype(np.float32)
    return pts[np.isfinite(pts).all(axis=1)]


# --------------------------------------------------------------------------
# voxel grid
# --------------------------------------------------------------------------
def voxel_keys(points, voxel):
    idx = np.floor(np.asarray(points, dtype=np.float64) / voxel).astype(np.int64) + _OFFSET
    return (idx[:, 0] << 42) | (idx[:, 1] << 21) | idx[:, 2]


def voxel_downsample(points, voxel):
    """Keep one point per voxel (the first one seen)."""
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    if len(points) == 0:
        return points
    _, idx = np.unique(voxel_keys(points, voxel), return_index=True)
    return points[np.sort(idx)]


# --------------------------------------------------------------------------
# PCD io (binary / ascii; binary_compressed is NOT supported)
# --------------------------------------------------------------------------
_PCD_HEADER = (
    "# .PCD v0.7 - Point Cloud Data file format\n"
    "VERSION 0.7\n"
    "FIELDS x y z intensity normal_x normal_y normal_z curvature\n"
    "SIZE 4 4 4 4 4 4 4 4\n"
    "TYPE F F F F F F F F\n"
    "COUNT 1 1 1 1 1 1 1 1\n"
    "WIDTH {n}\n"
    "HEIGHT 1\n"
    "VIEWPOINT 0 0 0 1 0 0 0\n"
    "POINTS {n}\n"
    "DATA binary\n"
)


def write_pcd(path, points):
    """Write binary PCD with the same 8 fields FAST-LIO2 uses (PointXYZINormal)."""
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    n = len(pts)
    data = np.zeros((n, 8), dtype=np.float32)
    data[:, :3] = pts
    with open(path, "wb") as f:
        f.write(_PCD_HEADER.format(n=n).encode("ascii"))
        f.write(data.tobytes())


_PCD_TYPES = {
    ("F", 4): "<f4", ("F", 8): "<f8",
    ("U", 1): "u1", ("U", 2): "<u2", ("U", 4): "<u4", ("U", 8): "<u8",
    ("I", 1): "i1", ("I", 2): "<i2", ("I", 4): "<i4", ("I", 8): "<i8",
}


def read_pcd_xyz(path):
    """Read x/y/z of an ASCII or binary PCD -> (N, 3) float32."""
    with open(path, "rb") as f:
        blob = f.read()
    header = {}
    pos = 0
    while True:
        end = blob.index(b"\n", pos)
        line = blob[pos:end].decode("ascii", "replace").strip()
        pos = end + 1
        if not line or line.startswith("#"):
            continue
        key, _, rest = line.partition(" ")
        header[key.upper()] = rest.split()
        if key.upper() == "DATA":
            break
    fields = header["FIELDS"]
    sizes = [int(s) for s in header["SIZE"]]
    types = header["TYPE"]
    counts = [int(c) for c in header.get("COUNT", ["1"] * len(fields))]
    npts = int(header["POINTS"][0])
    mode = header["DATA"][0].lower()
    if mode == "binary_compressed":
        raise ValueError("binary_compressed PCD is not supported; "
                         "re-save as binary (pcl_convert_pcd_ascii_binary)")
    if mode == "ascii":
        rows = np.loadtxt(blob[pos:].decode("ascii").splitlines(), ndmin=2)
        cols = [fields.index(a) for a in ("x", "y", "z")]
        return rows[:, cols].astype(np.float32)
    dt = []
    for i, name in enumerate(fields):
        fmt = _PCD_TYPES[(types[i], sizes[i])]
        if counts[i] > 1:
            dt.append(("%s_%d" % (name, i), fmt, (counts[i],)))
        else:
            dt.append(("%s_%d" % (name, i), fmt))
    arr = np.frombuffer(blob, dtype=np.dtype(dt), count=npts, offset=pos)
    out = np.stack([arr["%s_%d" % (a, fields.index(a))].astype(np.float32)
                    for a in ("x", "y", "z")], axis=1)
    return out[np.isfinite(out).all(axis=1)]


# --------------------------------------------------------------------------
# floor height (camera_init is z-down: the floor is the LARGE-z peak)
# --------------------------------------------------------------------------
def estimate_floor_z_down(points, bin_size=0.05):
    """z of the floor in a z-down cloud, or None if it cannot be found."""
    pts = np.asarray(points)
    if len(pts) < 100:
        return None
    z = pts[:, 2]
    lo, hi = np.percentile(z, [1, 99])
    if hi - lo < 4 * bin_size:
        return None
    mid = 0.5 * (lo + hi)
    edges = np.arange(lo, hi + bin_size, bin_size)
    hist, edges = np.histogram(z, bins=edges)
    centers = 0.5 * (edges[:-1] + edges[1:])
    masked = np.where(centers >= mid, hist, -1)
    k = int(np.argmax(masked))
    if masked[k] <= 0:
        return None
    return float(centers[k])


# --------------------------------------------------------------------------
# rigid transforms
# --------------------------------------------------------------------------
def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def rpy_to_R(roll, pitch, yaw):
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def make_T(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def invert_T(T):
    R = T[:3, :3]
    t = T[:3, 3]
    return make_T(R.T, -R.T @ t)


def apply_T(T, pts):
    return pts @ T[:3, :3].T + T[:3, 3]


def yaw_of(R):
    return math.atan2(R[1, 0], R[0, 0])


def R_to_quat(R):
    """3x3 rotation -> (x, y, z, w)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / n, y / n, z / n, w / n)


def quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# --------------------------------------------------------------------------
# registration (point-to-point ICP, numpy + scipy cKDTree)
# --------------------------------------------------------------------------
def kabsch(src, dst):
    cs, cd = src.mean(axis=0), dst.mean(axis=0)
    H = (src - cs).T @ (dst - cd)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    return R, cd - R @ cs


def icp(src, tree, dst, T0, max_iter=40, dist_schedule=(1.0, 0.6, 0.4, 0.3)):
    """Refine T so that apply_T(T, src) lies on dst. tree = cKDTree(dst)."""
    T = np.array(T0, dtype=np.float64)
    n_sched = len(dist_schedule)
    for it in range(max_iter):
        md = dist_schedule[min(it * n_sched // max_iter, n_sched - 1)]
        moved = apply_T(T, src)
        d, j = tree.query(moved, distance_upper_bound=md)
        ok = np.isfinite(d)
        if ok.sum() < 20:
            break
        R, t = kabsch(moved[ok], dst[j[ok]])
        T = make_T(R, t) @ T
        if np.linalg.norm(t) < 1e-5 and np.linalg.norm(R - np.eye(3)) < 1e-5:
            break
    return T


def inlier_stats(src, tree, T, inlier_dist):
    """(fraction of src within inlier_dist of the map, rmse of those inliers)."""
    if len(src) == 0:
        return 0.0, float("inf")
    d, _ = tree.query(apply_T(T, src), distance_upper_bound=inlier_dist)
    ok = np.isfinite(d)
    if not ok.any():
        return 0.0, float("inf")
    return float(ok.mean()), float(math.sqrt(np.mean(d[ok] ** 2)))


def _distinct(T_a, T_b, pos_tol, yaw_tol):
    e = invert_T(T_a) @ T_b
    return (np.linalg.norm(e[:3, 3]) > pos_tol) or (abs(yaw_of(e[:3, :3])) > yaw_tol)


def global_candidates(src, tree, dst, yaw_step_deg=15.0, trans_range=4.0,
                      trans_step=1.0, inlier_dist=0.5, max_score_pts=800,
                      top_k=8, nms_pos=0.75, nms_yaw_deg=20.0, seed=0):
    """Coarse search: yaw ring x xy grid around the centroid alignment, with NMS."""
    rng = np.random.RandomState(seed)
    if len(src) <= max_score_pts:
        sub = src
    else:
        sub = src[rng.choice(len(src), max_score_pts, replace=False)]
    cs, ct = src.mean(axis=0), dst.mean(axis=0)
    offs = np.arange(-trans_range, trans_range + 1e-9, trans_step)
    cands = []
    for yaw in np.deg2rad(np.arange(0.0, 360.0, yaw_step_deg)):
        R = rot_z(yaw)
        base = ct - R @ cs
        for dx in offs:
            for dy in offs:
                T = make_T(R, base + np.array([dx, dy, 0.0]))
                frac, _ = inlier_stats(sub, tree, T, inlier_dist)
                cands.append((frac, T))
    cands.sort(key=lambda c: -c[0])
    picked = []
    for _, T in cands:
        if all(_distinct(P, T, nms_pos, math.radians(nms_yaw_deg)) for P in picked):
            picked.append(T)
            if len(picked) >= top_k:
                break
    return picked


def register(src, dst, tree, init_Ts=None, final_inlier_dist=0.3, **global_kw):
    """Align src (live scan) onto dst (saved map).

    Returns (T, fitness, rmse, runner_up_fitness):
      T                   4x4, p_saved = T @ p_live
      fitness / rmse      inlier fraction and inlier RMSE at final_inlier_dist
      runner_up_fitness   fitness of the best DISTINCT alternative pose (0 if none);
                          close to `fitness` means the match is ambiguous.
    init_Ts : optional list of 4x4 seeds tried IN ADDITION to the global search.
    """
    seeds = list(init_Ts) if init_Ts else []
    seeds += global_candidates(src, tree, dst, **global_kw)
    results = []
    for T0 in seeds:
        T = icp(src, tree, dst, T0)
        frac, rmse = inlier_stats(src, tree, T, final_inlier_dist)
        results.append((round(frac, 3), -rmse, T, frac, rmse))
    results.sort(key=lambda r: (r[0], r[1]), reverse=True)
    _, _, T, frac, rmse = results[0]
    runner_up = 0.0
    for r in results[1:]:
        if _distinct(T, r[2], 0.5, math.radians(10.0)):
            runner_up = r[3]
            break
    return T, frac, rmse, runner_up
