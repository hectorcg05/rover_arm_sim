#!/usr/bin/env python3
"""Genera la piedra irregular del mundo de práctica (meshes/piedra.stl).

Es la envolvente convexa de puntos aleatorios sobre un elipsoide (semilla fija, siempre la misma
piedra). Convexa y con pocas caras para que la colisión en Gazebo sea estable y barata.

Uso: python3 tools/make_rock.py --out meshes/piedra.stl --size 0.12 0.10 0.08 --seed 7
"""

import argparse
import os
import sys

import numpy as np
from scipy.spatial import ConvexHull

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from rover_arm_sim.stl_io import write_stl  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--size', nargs=3, type=float, default=[0.12, 0.10, 0.08], help='ejes del elipsoide (m)')
    ap.add_argument('--points', type=int, default=28)
    ap.add_argument('--seed', type=int, default=7)
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)
    d = rng.normal(size=(args.points, 3))
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    pts = d * np.asarray(args.size) / 2 * rng.uniform(0.75, 1.0, size=(args.points, 1))
    pts -= pts.mean(axis=0)
    hull = ConvexHull(pts)
    tri = []
    for simplex, eq in zip(hull.simplices, hull.equations):
        t = pts[simplex]
        if np.dot(np.cross(t[1] - t[0], t[2] - t[0]), eq[:3]) < 0:
            t = t[[0, 2, 1]]
        tri.append(t)
    tri = np.array(tri)
    tri[:, :, 2] -= tri[:, :, 2].min()  # apoyada en z = 0 de su marco
    write_stl(args.out, tri)
    print(f'{args.out}: {len(tri)} caras, extensión {np.ptp(tri.reshape(-1, 3), axis=0).round(3)} m')


if __name__ == '__main__':
    main()
