#!/usr/bin/env python3
"""Compare saved GMapping maps with a reference map of the same world (no run log needed).

Aligns each map onto the reference (rigid SE(2)), then reports wall distance, precision and
recall, explored area, and saves an overlay image.

  python3 map_compare.py --ref office_ICTE4001_sim.yaml --maps gmapping_map.yaml gmapping_nav_map.yaml --out eval
"""
import argparse, os, sys, json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyse_run import GridMap, align_map, map_accuracy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ref', required=True)
    ap.add_argument('--maps', nargs='+', required=True)
    ap.add_argument('--tol', type=float, default=0.10)
    ap.add_argument('--out', default='map_eval')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    ref = GridMap(a.ref)
    rows = []
    for mp in a.maps:
        slam = GridMap(mp)
        T, c, dt, _ = align_map(slam, ref)
        acc, q, d = map_accuracy(slam, ref, T, dt, tol=a.tol)
        st = slam.stats()
        row = {'map': slam.name, **st, **acc}
        rows.append(row)
        ex = ref.extent()
        fig, ax = plt.subplots(figsize=(10, 4.6))
        img = np.where(ref.occupied, 0.0, np.where(ref.free, 1.0, 0.8))
        ax.imshow(img, cmap='gray', extent=ex, vmin=0, vmax=1)
        good = d <= a.tol
        ax.scatter(q[good, 0], q[good, 1], s=0.6, c='tab:green', label=f'GMapping wall within {a.tol:.2f} m')
        ax.scatter(q[~good, 0], q[~good, 1], s=0.6, c='tab:red', label='GMapping wall further away')
        ax.set_title(f'{slam.name} aligned on reference (precision {acc[f"precision_at_{a.tol:.2f}m_pct"]}%, '
                     f'recall {acc[f"recall_at_{a.tol:.2f}m_pct"]}%)')
        ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.legend(loc='lower left', markerscale=8, fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(a.out, os.path.splitext(slam.name)[0] + '_vs_reference.png'), dpi=160)
        plt.close(fig)
    with open(os.path.join(a.out, 'map_eval.json'), 'w') as f:
        json.dump(rows, f, indent=2)
    for r in rows:
        print(json.dumps(r, indent=2))


if __name__ == '__main__':
    main()
