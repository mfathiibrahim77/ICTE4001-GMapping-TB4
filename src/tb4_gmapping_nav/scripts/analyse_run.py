#!/usr/bin/env python3
"""
analyse_run - offline evaluation of a GMapping run (no ROS needed, only numpy/scipy/matplotlib/yaml).

    analyse_run.py --run ~/gmapping_runs/office1_20261020_101500 \
                   --map ~/maps/office1.yaml \
                   --ref $(ros2 pkg prefix tb4_gmapping_nav)/share/tb4_gmapping_nav/maps/reference/office_ICTE4001_sim.yaml

All inputs are optional except --run. Produces <run>/analysis/:
  trajectories.png     SLAM (map frame) vs odometry vs ground truth (if logged)
  pose_error.png       position / heading error over time (needs ground truth)
  uncertainty.png      weight entropy, N_eff (resampling), particle spread, explored area over time
  map_vs_reference.png saved SLAM map aligned on a reference map, walls coloured by error
  summary.md / summary.json  all numbers, ready to paste into the report

Metrics
  ATE (absolute trajectory error) = RMSE of position after the best rigid SE(2) alignment
      of the estimated trajectory onto ground truth (Umeyama/Kabsch, no scale).
  Final drift = position error at the end when both trajectories start at the same pose.
  Return-to-start error (--returned-to-start): the robot was physically brought back to its start
      pose (e.g. docked), so the true final pose is (0,0,0) in both map and odom frames.
  Map accuracy: SLAM occupied cells are aligned to the reference occupied cells by a rigid SE(2)
      transform (coarse FFT search + Nelder-Mead refinement of a truncated chamfer distance).
      Reported: mean/median/95th-percentile distance from SLAM walls to the nearest reference wall,
      precision@tol = % SLAM wall cells within tol of a reference wall,
      recall@tol    = % reference wall cells (inside the explored area) within tol of a SLAM wall.
"""
import argparse
import csv
import json
import math
import os

import numpy as np
import yaml

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from scipy import ndimage, optimize, signal  # noqa: E402


# ============================================================================ helpers
def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def se2_apply(T, xy):
    """T = (tx, ty, th). Apply rotation then translation to Nx2 points."""
    c, s = math.cos(T[2]), math.sin(T[2])
    R = np.array([[c, -s], [s, c]])
    return xy @ R.T + np.array([T[0], T[1]])


def umeyama_se2(src, dst):
    """Best rigid transform (no scale) mapping src -> dst (Nx2 each), least squares."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    H = (src - mu_s).T @ (dst - mu_d)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    th = math.atan2(R[1, 0], R[0, 0])
    t = mu_d - R @ mu_s
    return (t[0], t[1], th)


def start_align(est_xyth, ref_xyth):
    """Transform est so that its first pose coincides with ref's first pose."""
    dth = ref_xyth[0, 2] - est_xyth[0, 2]
    c, s = math.cos(dth), math.sin(dth)
    R = np.array([[c, -s], [s, c]])
    xy = (est_xyth[:, :2] - est_xyth[0, :2]) @ R.T + ref_xyth[0, :2]
    return np.column_stack([xy, wrap(est_xyth[:, 2] + dth)])


def path_length(xy):
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1))) if len(xy) > 1 else 0.0


def load_csv(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    cols = {}
    for k in rows[0].keys() if rows else []:
        vals = []
        for r in rows:
            try:
                vals.append(float(r[k]))
            except (TypeError, ValueError):
                vals.append(np.nan)
        cols[k] = np.array(vals)
    return cols


# ============================================================================ maps
def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    # header: magic, width, height, maxval (comments allowed)
    tokens, i = [], 0
    while len(tokens) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while data[i:i + 1] not in (b'\n', b''):
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    i += 1  # single whitespace after maxval
    if magic == b'P5':
        dtype = np.uint8 if maxval < 256 else '>u2'
        img = np.frombuffer(data, dtype=dtype, count=w * h, offset=i).reshape(h, w)
    elif magic == b'P2':
        img = np.array(data[i:].split(), dtype=int)[:w * h].reshape(h, w)
    else:
        raise ValueError(f'{path}: only PGM (P5/P2) maps are supported, got {magic!r}')
    return img.astype(np.float64) / maxval


class GridMap:
    """ROS map_server map: occ[row, col] with row 0 = TOP of the image (max y)."""

    def __init__(self, yaml_path):
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        img_path = meta['image']
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)
        img = read_pgm(img_path)
        p = img if meta.get('negate', 0) else 1.0 - img      # occupancy probability
        self.res = float(meta['resolution'])
        self.origin = [float(v) for v in meta['origin']]
        self.occupied = p > float(meta.get('occupied_thresh', 0.65))
        self.free = p < float(meta.get('free_thresh', 0.25))
        # map_saver (trinary) writes unknown cells as grey 205 -> p = 0.196, which is below
        # Nav2's default free_thresh 0.25, so treat that exact value as unknown explicitly.
        if meta.get('mode', 'trinary') == 'trinary':
            self.free &= np.rint(img * 255).astype(int) != 205
        self.unknown = ~(self.occupied | self.free)
        self.h, self.w = img.shape
        self.name = os.path.basename(yaml_path)

    def cells_to_world(self, rows, cols):
        x = self.origin[0] + (cols + 0.5) * self.res
        y = self.origin[1] + (self.h - rows - 0.5) * self.res
        if abs(self.origin[2]) > 1e-9:
            return se2_apply((self.origin[0], self.origin[1], self.origin[2]),
                             np.column_stack([x - self.origin[0], y - self.origin[1]]))
        return np.column_stack([x, y])

    def world_to_cells(self, xy):
        if abs(self.origin[2]) > 1e-9:
            c, s = math.cos(-self.origin[2]), math.sin(-self.origin[2])
            d = xy - np.array(self.origin[:2])
            xy = d @ np.array([[c, -s], [s, c]]).T + np.array(self.origin[:2])
        cols = np.floor((xy[:, 0] - self.origin[0]) / self.res).astype(int)
        rows = self.h - 1 - np.floor((xy[:, 1] - self.origin[1]) / self.res).astype(int)
        return rows, cols

    def occupied_points(self):
        r, c = np.nonzero(self.occupied)
        return self.cells_to_world(r, c)

    def extent(self):
        x0, y0 = self.origin[0], self.origin[1]
        return [x0, x0 + self.w * self.res, y0, y0 + self.h * self.res]

    def stats(self):
        a = self.res ** 2
        return {'map': self.name, 'size_cells': [int(self.w), int(self.h)], 'resolution_m': self.res,
                'known_area_m2': round(float((~self.unknown).sum() * a), 2),
                'free_area_m2': round(float(self.free.sum() * a), 2),
                'occupied_cells': int(self.occupied.sum())}


def align_map(slam, ref, trunc=0.5, init=None, n_pts=4000, coarse_res=0.1, ang_step_deg=2.0):
    """Rigid SE(2) aligning SLAM occupied points onto the reference occupied cells."""
    P = slam.occupied_points()
    if len(P) == 0:
        raise ValueError('SLAM map has no occupied cells')
    rng = np.random.default_rng(0)
    P_sub = P[rng.choice(len(P), min(n_pts, len(P)), replace=False)]

    # distance (m) from every reference cell to the nearest reference wall
    dt = ndimage.distance_transform_edt(~ref.occupied) * ref.res

    def cost(T, pts=P_sub):
        q = se2_apply(T, pts)
        r, c = ref.world_to_cells(q)
        inside = (r >= 0) & (r < ref.h) & (c >= 0) & (c < ref.w)
        d = np.full(len(q), trunc)
        d[inside] = np.minimum(dt[r[inside], c[inside]], trunc)
        return float(np.mean(d))

    candidates = []
    if init is not None:
        candidates.append(tuple(init))
    else:
        # coarse search: for each rotation, best translation by FFT cross-correlation
        ex = ref.extent()
        ref_score_full = np.exp(-(dt ** 2) / (2 * 0.15 ** 2))
        k = max(1, int(round(coarse_res / ref.res)))
        ref_score = ref_score_full[::k, ::k]
        cres = ref.res * k
        Hc, Wc = ref_score.shape
        Pc = P_sub - P_sub.mean(0)
        results = []
        for th in np.deg2rad(np.arange(0.0, 360.0, ang_step_deg)):
            q = se2_apply((0.0, 0.0, th), Pc)
            span = np.ptp(q, axis=0)
            hw = int(np.ceil(span[0] / cres)) + 3
            hh = int(np.ceil(span[1] / cres)) + 3
            img = np.zeros((hh, hw))
            qx0, qy1 = q[:, 0].min(), q[:, 1].max()
            cc = np.clip(((q[:, 0] - qx0) / cres).astype(int) + 1, 0, hw - 1)
            rr = np.clip(((qy1 - q[:, 1]) / cres).astype(int) + 1, 0, hh - 1)
            np.add.at(img, (rr, cc), 1.0)
            corr = signal.fftconvolve(ref_score, img[::-1, ::-1], mode='full')
            idx = np.unravel_index(np.argmax(corr), corr.shape)
            # top-left of img placed at ref coarse cell (row0, col0)
            row0 = idx[0] - (hh - 1)
            col0 = idx[1] - (hw - 1)
            # world coordinates of img pixel (1,1) <-> (qx0, qy1) after placement
            wx = ex[0] + (col0 + 1 + 0.5) * cres
            wy = ex[3] - (row0 + 1 + 0.5) * cres
            tx, ty = wx - qx0, wy - qy1
            # convert translation for centred points back to original points
            off = se2_apply((0.0, 0.0, th), -P_sub.mean(0)[None, :])[0]
            results.append((corr[idx], (tx + off[0], ty + off[1], th)))
        results.sort(key=lambda r: -r[0])
        candidates = [r[1] for r in results[:6]]

    best = None
    for T0 in candidates:
        res = optimize.minimize(lambda v: cost(v), np.array(T0), method='Nelder-Mead',
                                options={'xatol': 1e-3, 'fatol': 1e-5, 'maxiter': 600,
                                         'initial_simplex': np.array(T0) + np.array(
                                             [[0, 0, 0], [0.2, 0, 0], [0, 0.2, 0], [0, 0, 0.05]])})
        if best is None or res.fun < best[1]:
            best = (tuple(res.x), res.fun)
    return best[0], best[1], dt, cost


def map_accuracy(slam, ref, T, dt, tol=0.10, trunc_report=1.0):
    P = slam.occupied_points()
    q = se2_apply(T, P)
    r, c = ref.world_to_cells(q)
    inside = (r >= 0) & (r < ref.h) & (c >= 0) & (c < ref.w)
    d = np.full(len(q), np.inf)
    d[inside] = dt[r[inside], c[inside]]
    d_fin = d[np.isfinite(d)]

    # recall: reference walls inside the explored (known) region of the SLAM map
    slam_dt = ndimage.distance_transform_edt(~slam.occupied) * slam.res
    known_dil = ndimage.binary_dilation(~slam.unknown, iterations=3)
    rr, cc = np.nonzero(ref.occupied)
    Q = ref.cells_to_world(rr, cc)
    Tinv = (0, 0, -T[2])
    Q_s = se2_apply(Tinv, Q - np.array(T[:2]))
    sr, sc = slam.world_to_cells(Q_s)
    ins = (sr >= 0) & (sr < slam.h) & (sc >= 0) & (sc < slam.w)
    ins_idx = np.nonzero(ins)[0]
    explored = ins_idx[known_dil[sr[ins_idx], sc[ins_idx]]]
    d_ref = slam_dt[sr[explored], sc[explored]]

    out = {
        'alignment_tx_ty_yawdeg': [round(float(T[0]), 3), round(float(T[1]), 3),
                                   round(math.degrees(float(T[2])), 2)],
        'n_slam_wall_cells': int(len(P)),
        'wall_dist_mean_m': round(float(np.mean(np.minimum(d_fin, trunc_report))), 4),
        'wall_dist_median_m': round(float(np.median(d_fin)), 4),
        'wall_dist_p95_m': round(float(np.percentile(d_fin, 95)), 4),
        f'precision_at_{tol:.2f}m_pct': round(100.0 * float(np.mean(d <= tol)), 1),
        f'recall_at_{tol:.2f}m_pct': (round(100.0 * float(np.mean(d_ref <= tol)), 1)
                                     if len(d_ref) else None),
        'n_ref_wall_cells_in_explored_area': int(len(d_ref)),
    }
    return out, q, d


# ============================================================================ main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--run', required=True, help='directory written by slam_eval_recorder')
    ap.add_argument('--map', help='saved GMapping map (.yaml from map_saver_cli)')
    ap.add_argument('--ref', help='reference / ground-truth map (.yaml)')
    ap.add_argument('--ref-init', nargs=3, type=float, metavar=('TX', 'TY', 'YAW_RAD'),
                    help='initial guess for the SLAM->reference alignment (skips the global search)')
    ap.add_argument('--tol', type=float, default=0.10, help='[m] tolerance for precision/recall')
    ap.add_argument('--returned-to-start', action='store_true',
                    help='robot was physically returned to its start pose at the end')
    ap.add_argument('--out', help='output directory (default <run>/analysis)')
    ap.add_argument('--particles', type=int, default=50, help='N used in the run (for the N_eff plot)')
    ap.add_argument('--resample-threshold', type=float, default=0.5)
    args = ap.parse_args()

    out = args.out or os.path.join(args.run, 'analysis')
    os.makedirs(out, exist_ok=True)
    summary = {'run': os.path.abspath(args.run)}

    # ---------------------------------------------------------------- trajectories
    poses_path = os.path.join(args.run, 'poses.csv')
    if os.path.exists(poses_path):
        P = load_csv(poses_path)
        t = P['t'] - P['t'][0]
        slam = np.column_stack([P['slam_x'], P['slam_y'], P['slam_yaw']])
        odom = np.column_stack([P['odom_x'], P['odom_y'], P['odom_yaw']])
        odom_in_map = start_align(odom, slam)            # odometry drawn from the same start pose
        traj = {'duration_s': round(float(t[-1]), 1),
                'path_length_slam_m': round(path_length(slam[:, :2]), 2),
                'path_length_odom_m': round(path_length(odom[:, :2]), 2),
                'final_slam_vs_odom_discrepancy_m': round(float(np.linalg.norm(
                    slam[-1, :2] - odom_in_map[-1, :2])), 3),
                'final_slam_vs_odom_heading_deg': round(float(np.degrees(abs(wrap(
                    slam[-1, 2] - odom_in_map[-1, 2])))), 2)}

        has_gt = 'gt_x' in P and np.isfinite(P['gt_x']).sum() > 10
        fig, ax = plt.subplots(figsize=(7, 7))
        if has_gt:
            m = np.isfinite(P['gt_x'])
            gt = np.column_stack([P['gt_x'], P['gt_y'], P['gt_yaw']])[m]
            s_m, o_m, t_m = slam[m], odom[m], t[m]
            # ATE: best SE(2) fit of each estimate onto ground truth
            res = {}
            for name, est in (('slam', s_m), ('odom', o_m)):
                T = umeyama_se2(est[:, :2], gt[:, :2])
                e_xy = np.linalg.norm(se2_apply(T, est[:, :2]) - gt[:, :2], axis=1)
                e_th = np.abs(wrap(est[:, 2] + T[2] - gt[:, 2]))
                sa = start_align(est, gt)
                res[name] = dict(T=T, e_xy=e_xy, e_th=e_th, sa=sa)
                traj[f'ATE_rmse_{name}_m'] = round(float(np.sqrt(np.mean(e_xy ** 2))), 4)
                traj[f'ATE_max_{name}_m'] = round(float(np.max(e_xy)), 4)
                traj[f'heading_rmse_{name}_deg'] = round(float(np.degrees(np.sqrt(np.mean(e_th ** 2)))), 3)
                traj[f'final_drift_{name}_m'] = round(float(np.linalg.norm(sa[-1, :2] - gt[-1, :2])), 4)
            ax.plot(gt[:, 0], gt[:, 1], 'k-', lw=2.5, label='ground truth')
            ax.plot(res['slam']['sa'][:, 0], res['slam']['sa'][:, 1], '-', c='tab:blue', lw=1.5,
                    label=f"GMapping (ATE {traj['ATE_rmse_slam_m']:.3f} m)")
            ax.plot(res['odom']['sa'][:, 0], res['odom']['sa'][:, 1], '--', c='tab:red', lw=1.2,
                    label=f"odometry (ATE {traj['ATE_rmse_odom_m']:.3f} m)")
            ax.set_title('Trajectories (world frame, start-aligned)')

            fig2, ax2 = plt.subplots(2, 1, figsize=(9, 5.5), sharex=True)
            ax2[0].plot(t_m, res['slam']['e_xy'], c='tab:blue', label='GMapping')
            ax2[0].plot(t_m, res['odom']['e_xy'], c='tab:red', ls='--', label='odometry')
            ax2[0].set_ylabel('position error [m]')
            ax2[0].legend()
            ax2[0].grid(alpha=0.3)
            ax2[1].plot(t_m, np.degrees(res['slam']['e_th']), c='tab:blue')
            ax2[1].plot(t_m, np.degrees(res['odom']['e_th']), c='tab:red', ls='--')
            ax2[1].set_ylabel('heading error [deg]')
            ax2[1].set_xlabel('time [s]')
            ax2[1].grid(alpha=0.3)
            fig2.suptitle('Pose error vs ground truth (after SE(2) alignment)')
            fig2.tight_layout()
            fig2.savefig(os.path.join(out, 'pose_error.png'), dpi=150)
            plt.close(fig2)
        else:
            ax.plot(slam[:, 0], slam[:, 1], '-', c='tab:blue', lw=1.8, label='GMapping (map frame)')
            ax.plot(odom_in_map[:, 0], odom_in_map[:, 1], '--', c='tab:red', lw=1.2,
                    label='odometry (start-aligned)')
            ax.set_title('Trajectories (no ground truth logged)')
        if args.returned_to_start:
            traj['return_to_start_error_slam_m'] = round(float(np.linalg.norm(slam[-1, :2] - slam[0, :2])), 4)
            traj['return_to_start_error_odom_m'] = round(float(np.linalg.norm(odom[-1, :2] - odom[0, :2])), 4)
            traj['return_to_start_heading_slam_deg'] = round(float(np.degrees(abs(wrap(slam[-1, 2] - slam[0, 2])))), 2)
            traj['return_to_start_heading_odom_deg'] = round(float(np.degrees(abs(wrap(odom[-1, 2] - odom[0, 2])))), 2)
        ax.plot(slam[0, 0], slam[0, 1], 'go', ms=9, label='start')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.axis('equal')
        ax.grid(alpha=0.3)
        ax.legend(loc='best', fontsize=9)
        fig.tight_layout()
        fig.savefig(os.path.join(out, 'trajectories.png'), dpi=150)
        plt.close(fig)

        # uncertainty
        ml_path = os.path.join(args.run, 'map_log.csv')
        ML = load_csv(ml_path) if os.path.exists(ml_path) and os.path.getsize(ml_path) > 80 else None
        fig, ax = plt.subplots(4, 1, figsize=(9, 9), sharex=True)
        ax[0].plot(t, P['entropy'], c='tab:purple')
        ax[0].set_ylabel('weight entropy [nats]')
        if 'neff' in P and np.isfinite(P['neff']).any():
            ax[1].plot(t, P['neff'], c='tab:blue', label='$N_{eff}$')
            if args.particles:
                ax[1].axhline(args.resample_threshold * args.particles, c='k', ls=':',
                              label=f'resample threshold ({args.resample_threshold:g}·N)')
            ax[1].legend(loc='lower right', fontsize=8)
            nf = P['neff'][np.isfinite(P['neff'])]
            traj['neff_mean'] = round(float(np.mean(nf)), 2)
            traj['neff_min'] = round(float(np.min(nf)), 2)
        ax[1].set_ylabel('$N_{eff}$')
        ax[2].plot(t, P['particle_spread'], c='tab:orange')
        ax[2].set_ylabel('particle spread [m]')
        if ML is not None and len(ML.get('t', [])):
            ax[3].plot(ML['t'] - P['t'][0], ML['known_area_m2'], c='tab:green', marker='.')
            traj['final_known_area_m2'] = round(float(ML['known_area_m2'][-1]), 2)
            traj['n_map_updates'] = int(len(ML['t']))
        ax[3].set_ylabel('explored area [m²]')
        ax[3].set_xlabel('time [s]')
        for a in ax:
            a.grid(alpha=0.3)
        fig.suptitle('Filter uncertainty and map growth')
        fig.tight_layout()
        fig.savefig(os.path.join(out, 'uncertainty.png'), dpi=150)
        plt.close(fig)
        ent = P['entropy'][np.isfinite(P['entropy'])]
        spr = P['particle_spread'][np.isfinite(P['particle_spread'])]
        if len(ent):
            traj['entropy_mean'] = round(float(np.mean(ent)), 3)
        if len(spr):
            traj['particle_spread_mean_m'] = round(float(np.mean(spr)), 4)
            traj['particle_spread_max_m'] = round(float(np.max(spr)), 4)
        summary['trajectory'] = traj
    else:
        print(f'(no {poses_path}, skipping trajectory analysis)')

    # ---------------------------------------------------------------- maps
    if args.map:
        slam_map = GridMap(args.map)
        summary['slam_map'] = slam_map.stats()
        if args.ref:
            ref = GridMap(args.ref)
            summary['reference_map'] = ref.stats()
            T, c, dt, _ = align_map(slam_map, ref, init=args.ref_init)
            acc, q, d = map_accuracy(slam_map, ref, T, dt, tol=args.tol)
            acc['alignment_cost_m'] = round(c, 4)
            summary['map_accuracy'] = acc

            fig, ax = plt.subplots(figsize=(9, 8))
            ax.imshow(np.where(ref.occupied, 0.15, np.where(ref.free, 1.0, 0.8)), cmap='gray',
                      vmin=0, vmax=1, extent=ref.extent(), origin='upper', interpolation='nearest')
            dd = np.clip(np.where(np.isfinite(d), d, 1.0), 0, 0.3)
            sc = ax.scatter(q[:, 0], q[:, 1], c=dd, s=1.5, cmap='turbo', vmin=0, vmax=0.3)
            plt.colorbar(sc, ax=ax, fraction=0.04, label='distance to reference wall [m]')
            ax.set_title(f'GMapping map aligned on {ref.name}\n'
                         f"mean wall error {acc['wall_dist_mean_m']:.3f} m, "
                         f"precision@{args.tol:.2f}m {acc[f'precision_at_{args.tol:.2f}m_pct']}%")
            ax.set_xlabel('x [m]')
            ax.set_ylabel('y [m]')
            fig.tight_layout()
            fig.savefig(os.path.join(out, 'map_vs_reference.png'), dpi=170)
            plt.close(fig)
        else:
            fig, ax = plt.subplots(figsize=(8, 8))
            ax.imshow(np.where(slam_map.occupied, 0.0, np.where(slam_map.free, 1.0, 0.75)), cmap='gray',
                      extent=slam_map.extent(), origin='upper', interpolation='nearest')
            ax.set_title(f'GMapping map {slam_map.name}')
            fig.tight_layout()
            fig.savefig(os.path.join(out, 'map.png'), dpi=170)
            plt.close(fig)

    # ---------------------------------------------------------------- write
    with open(os.path.join(out, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=2)
    lines = ['# GMapping run summary', '', f"Run: `{summary['run']}`", '']
    for sec in ('trajectory', 'slam_map', 'reference_map', 'map_accuracy'):
        if sec in summary:
            lines += [f'## {sec}', '', '| metric | value |', '|---|---|']
            lines += [f'| {k} | {v} |' for k, v in summary[sec].items()]
            lines.append('')
    with open(os.path.join(out, 'summary.md'), 'w') as f:
        f.write('\n'.join(lines))
    print('\n'.join(lines))
    print(f'\nFigures and summary written to {out}')


if __name__ == '__main__':
    main()
