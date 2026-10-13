"""Helpers of tools/map_quality.py: placed points and trajectory of a result folder, and the map-quality measures.

Needs numpy (and pymupdf for the plan overlay). The chunk LAZ files are exported to ASCII with CloudCompare once
(scan_lio_<n>.asc next to them, as spot_check.py does) and placed with session_poses.mrp; the placed points are cached in
<result>/map_quality_cache/points.npy (x, y, z, time [s], every 2nd point).

Recording 44 (Haus 47, LG; plan C:/Users/User/Downloads/dLidar/reference/haus47_LG_plan.pdf) is described by the
constants below; for other recordings only the plan-free measures (revisit consistency without the pair list, jitter)
are meaningful.
"""
import glob
import os
import subprocess

import numpy as np

import spot_check
import trajectory_seams

# ---- recording 44 --------------------------------------------------------------------------------------------------
ROOM_ANGLE_DEG = 10.55                       # direction of the main block's walls in the map [deg]
FACADE_WINDOWS = {                           # time window [s] and band along the gallery w [m] in which the facade is fitted
    "start": (0.9, 5.9, -8.0, 6.0),          # seen through the windows of the start room
    "gallery_out": (48.9, 59.9, 4.0, 20.0),  # along the gallery, way out
    "gallery_back": (59.9, 70.9, 4.0, 20.0),  # way back
}
FACADE_U_BAND = (1.0, 4.5)                   # band across the gallery [m] searched for the facade
# revisit pairs (slice A start, slice B start) [s, point time]: places passed twice, registered on 2.5 s slices
REVISIT_PAIRS = [(40, 75), (45, 70), (50, 65), (30, 82.5), (35, 77.5), (27.5, 225), (32.5, 230), (85, 222.5),
                 (30, 265), (17.5, 270), (225, 267.5)]
HEIGHT_BAND = (-0.5, 1.5)                    # world z band of the floor-0 points used for walls [m]
PLAN_PDF = r"C:/Users/User/Downloads/dLidar/reference/haus47_LG_plan.pdf"
PLAN_ROTATION_DEG = 259.45                   # map -> plan rotation (main block at 10.55 deg in the map, 0 on the plan)
MAP_SHAFT = (-4.7, -4.9)                     # stairwell centre in the map [m]
PLAN_SHAFT_PX = (2101.0, 2917.0)             # its centre on the plan rendered at 150 dpi [px]
PLAN_PX_PER_M = 150 / 25.4 * 20.0            # 1:50 at 150 dpi


# ---- loading ---------------------------------------------------------------------------------------------------------
def cache_dir(result):
    d = os.path.join(result, "map_quality_cache")
    os.makedirs(d, exist_ok=True)
    return d


def points(result):
    """placed points (x, y, z, t) of a result, every 2nd point, cached"""
    c = os.path.join(cache_dir(result), "points.npy")
    if os.path.exists(c):
        return np.load(c)
    out = []
    for laz, M in spot_check.chunk_poses(result).items():
        asc = os.path.join(result, laz.replace(".laz", ".asc"))
        if not os.path.exists(asc):
            subprocess.run([spot_check.CC, "-SILENT", "-AUTO_SAVE", "OFF", "-O", os.path.join(result, laz), "-C_EXPORT_FMT", "ASC",
                            "-SEP", "SPACE", "-PREC", "4", "-SAVE_CLOUDS", "FILE", '"' + asc + '"'], check=True, capture_output=True)
        a = np.fromfile(asc, sep=" ").reshape(-1, 5)[::2]
        M = np.array(M)
        g = a[:, :3] @ M[:3, :3].T + M[:3, 3]
        out.append(np.column_stack([g, a[:, 4] * 1e-9]))
    p = np.concatenate(out)
    np.save(c, p)
    return p


def trajectory(result):
    """time [s] (sorted), world rotations (n x 3 x 3) and positions (n x 3) of every node"""
    T, Rs, P = [], [], []
    for fn, m in trajectory_seams.chunk_poses(result):
        M = np.array(m)
        a = np.loadtxt(os.path.join(result, fn), skiprows=1, usecols=range(13), ndmin=2)
        T.append(a[:, 0] * 1e-9)
        Rs.append(np.einsum("ij,njk->nik", M[:3, :3], a[:, [1, 2, 3, 5, 6, 7, 9, 10, 11]].reshape(-1, 3, 3)))
        P.append(a[:, [4, 8, 12]] @ M[:3, :3].T + M[:3, 3])
    T = np.concatenate(T)
    o = np.argsort(T)
    return T[o], np.concatenate(Rs)[o], np.concatenate(P)[o]


def room_frame(xy):
    """u across the gallery, w along it (room frame of recording 44)"""
    a = np.radians(ROOM_ANGLE_DEG)
    c, s = np.cos(a), np.sin(a)
    return c * xy[:, 0] + s * xy[:, 1], -s * xy[:, 0] + c * xy[:, 1]


# ---- global geometry: the facade at the start and along the gallery -------------------------------------------------
def _ransac_line(u, w, thr=0.03, iters=4000, rng=None):
    rng = rng or np.random.default_rng(1)
    idx = np.arange(len(u))
    best = None
    for _ in range(iters):
        i, j = rng.choice(idx, 2, replace=False)
        dw, du = w[j] - w[i], u[j] - u[i]
        if abs(dw) < 1.0 or abs(du / dw) > np.tan(np.radians(8)):
            continue
        n = np.array([1.0, -du / dw])
        n /= np.linalg.norm(n)
        inl = idx[np.abs(n[0] * (u - u[i]) + n[1] * (w - w[i])) < thr]
        if best is None or len(inl) > len(best):
            best = inl
    if best is None:
        return None
    P = np.column_stack([w[best], u[best]])
    m = P.mean(0)
    _, _, vt = np.linalg.svd(P - m, full_matrices=False)  # the full U would be N x N (tens of GB)
    ang = (np.degrees(np.arctan2(vt[0, 1], vt[0, 0])) + 90) % 180 - 90
    return ang, m[1], len(best)


def facade_angles(pts):
    """main facade line angle [deg] in each FACADE_WINDOWS entry (None where the facade is not seen)"""
    z_ok = (pts[:, 2] > HEIGHT_BAND[0]) & (pts[:, 2] < HEIGHT_BAND[1])
    u, w = room_frame(pts[:, :2])
    out = {}
    for name, (t0, t1, w0, w1) in FACADE_WINDOWS.items():
        m = z_ok & (pts[:, 3] >= t0) & (pts[:, 3] < t1) & (w > w0) & (w < w1) & (u > FACADE_U_BAND[0]) & (u < FACADE_U_BAND[1])
        sel = np.flatnonzero(m)
        if len(sel) < 300:
            out[name] = None
            continue
        if len(sel) > 60000:
            sel = np.random.default_rng(0).choice(sel, 60000, replace=False)
        out[name] = _ransac_line(u[sel], w[sel])
    return out


# ---- consistency: revisits of the same place -------------------------------------------------------------------------
def _raster(q, c, rad, res, n):
    ij = np.floor((q - c + rad + 2.0) / res).astype(int)
    ok = (ij >= 0).all(1) & (ij < n).all(1)
    img = np.zeros((n, n))
    np.add.at(img, (ij[ok, 0], ij[ok, 1]), 1.0)
    img = np.log1p(img)
    return img - img.mean()


def revisit(pts, T, P, ta, tb, step=2.5, rad=7.0, res=0.05, maxrot=8.0, drot=0.2):
    """heading [deg] and offset [m] of slice B against slice A where both see the same place (2D registration of the
    floor-0 points within rad of the place), and the peak quality (best score / best score >= 2 deg away)"""
    def sensor(a):
        m = (T >= a) & (T < a + step)
        return P[m, :2][::20]
    pa, pb = sensor(ta), sensor(tb)
    if len(pa) == 0 or len(pb) == 0:
        return None
    d = np.linalg.norm(pa[:, None] - pb[None], axis=2)
    i, j = np.unravel_index(np.argmin(d), d.shape)
    c = (pa[i] + pb[j]) / 2
    z_ok = (pts[:, 2] > HEIGHT_BAND[0]) & (pts[:, 2] < HEIGHT_BAND[1])

    def local(a):
        m = z_ok & (pts[:, 3] >= a) & (pts[:, 3] < a + step)
        q = pts[m, :2]
        return q[np.linalg.norm(q - c, axis=1) < rad]
    qa, qb = local(ta), local(tb)
    if len(qa) < 3000 or len(qb) < 3000:
        return None
    n = int(2 * (rad + 2.0) / res)
    FA = np.fft.fft2(_raster(qa, c, rad, res, n))
    rots = np.arange(-maxrot, maxrot + 1e-9, drot)
    scores, shifts = [], []
    for r in rots:
        th = np.radians(r)
        Rm = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        cc = np.real(np.fft.ifft2(FA * np.conj(np.fft.fft2(_raster((qb - c) @ Rm.T + c, c, rad, res, n)))))
        k = np.unravel_index(np.argmax(cc), cc.shape)
        scores.append(cc[k])
        shifts.append(k)
    scores = np.array(scores)
    i = int(np.argmax(scores))
    fr = 0.0
    if 0 < i < len(scores) - 1:
        den = scores[i - 1] - 2 * scores[i] + scores[i + 1]
        fr = 0.5 * (scores[i - 1] - scores[i + 1]) / den if den else 0.0
    far = np.abs(rots - rots[i]) >= 2.0
    quality = scores[i] / scores[far].max() if far.any() else np.nan
    k = shifts[i]
    offset = np.hypot((k[0] + n // 2) % n - n // 2, (k[1] + n // 2) % n - n // 2) * res
    return -(rots[i] + fr * drot), offset, quality


# ---- sharpness: pose jitter ------------------------------------------------------------------------------------------
def jitter(Rs, k=10):
    """rotation angle over k consecutive nodes [deg]: p50, p99"""
    d = np.einsum("nji,njk->nik", Rs[:-k], Rs[k:])
    ang = np.degrees(np.arccos(np.clip((np.trace(d, axis1=1, axis2=2) - 1) / 2, -1, 1)))
    return np.percentile(ang, 50), np.percentile(ang, 99)


# ---- plan overlay ----------------------------------------------------------------------------------------------------
def plan_overlay(pts, png, t_max=100.0, s=4, crop=(360, 430, 1220, 1010)):
    """the points of the first t_max s (floor-0 band) in red over the plan, anchored at the stairwell centre"""
    import pymupdf
    doc = pymupdf.open(PLAN_PDF)
    pix = doc[0].get_pixmap(dpi=150, colorspace=pymupdf.csGRAY)
    A = np.frombuffer(pix.samples, np.uint8).reshape(pix.height, pix.width)
    H = A.shape[0]
    th = np.radians(PLAN_ROTATION_DEG)
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    plan_shaft = np.array([PLAN_SHAFT_PX[0] / PLAN_PX_PER_M, (H - PLAN_SHAFT_PX[1]) / PLAN_PX_PER_M])
    t = plan_shaft - R @ np.array(MAP_SHAFT)
    m = (pts[:, 3] < t_max) & (pts[:, 2] > HEIGHT_BAND[0]) & (pts[:, 2] < HEIGHT_BAND[1])
    q = pts[m][::6, :2] @ R.T + t
    img = np.stack([A[::s, ::s]] * 3, -1).copy()
    col = (q[:, 0] * PLAN_PX_PER_M / s).astype(int)
    row = ((H - q[:, 1] * PLAN_PX_PER_M) / s).astype(int)
    ok = (col >= 0) & (col < img.shape[1]) & (row >= 0) & (row < img.shape[0])
    img[row[ok], col[ok]] = [230, 0, 0]
    out = np.ascontiguousarray(img[crop[1]:crop[3], crop[0]:crop[2]]).astype(np.uint8)
    h, w, _ = out.shape
    pymupdf.Pixmap(pymupdf.csRGB, w, h, out.tobytes(), False).save(png)
