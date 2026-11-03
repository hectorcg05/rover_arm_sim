#!/usr/bin/env python3
"""Distancias entre las colisiones del modelo generado (cajas y cilindros) para una postura.

Lee el URDF que genera el launch (~/.ros/rover_arm_sim/rover_arm.urdf), hace la cinemática directa
con los ángulos dados (convención de arm_manager, grados) y reporta la distancia mínima entre cada
eslabón móvil y: el chasis/llantas/base, los eslabones no contiguos y el piso (z = 0).

Uso: python3 tools/check_clearance.py 0 170 -146 0 0 [--urdf ruta] [--step 0.005]
"""

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from rover_arm_sim.geometry import axis_angle, homogeneous, origin_matrix, parse_floats, transform_points  # noqa
from rover_arm_sim import model_builder as mb  # noqa: E402


def box_points(size, step):
    pts = []
    s = np.asarray(size) / 2.0
    for axis in range(3):
        o = [i for i in range(3) if i != axis]
        n = [max(2, int(2 * s[i] / step) + 1) for i in o]
        u, v = np.meshgrid(np.linspace(-s[o[0]], s[o[0]], n[0]), np.linspace(-s[o[1]], s[o[1]], n[1]))
        for sign in (-1, 1):
            p = np.zeros((u.size, 3))
            p[:, o[0]] = u.ravel()
            p[:, o[1]] = v.ravel()
            p[:, axis] = sign * s[axis]
            pts.append(p)
    return np.vstack(pts)


def cylinder_points(radius, length, step):
    n = max(8, int(2 * math.pi * radius / step))
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    zs = np.linspace(-length / 2, length / 2, max(2, int(length / step) + 1))
    side = np.array([[radius * math.cos(t), radius * math.sin(t), z] for z in zs for t in a])
    return side


def link_collision_points(link, step):
    pts = []
    for col in link.findall('collision'):
        t = origin_matrix(col.find('origin'))
        g = col.find('geometry')
        if g.find('box') is not None:
            p = box_points(parse_floats(g.find('box').get('size')), step)
        elif g.find('cylinder') is not None:
            c = g.find('cylinder')
            p = cylinder_points(float(c.get('radius')), float(c.get('length')), step)
        else:
            continue
        pts.append(transform_points(t, p))
    return np.vstack(pts) if pts else np.zeros((0, 3))


def forward(robot, joint_values):
    """Pose de cada link en el mundo. joint_values: {nombre_joint: valor (rad o m)}."""
    joints = {j.find('child').get('link'): j for j in robot.findall('joint')}
    poses = {}

    def pose(link):
        if link in poses:
            return poses[link]
        j = joints.get(link)
        if j is None:
            poses[link] = np.eye(4)
            return poses[link]
        t = pose(j.find('parent').get('link')) @ origin_matrix(j.find('origin'))
        q = joint_values.get(j.get('name'), 0.0)
        axis = parse_floats(j.find('axis').get('xyz')) if j.find('axis') is not None else [0, 0, 1]
        if j.get('type') == 'revolute':
            t = t @ homogeneous(axis_angle(axis, q))
        elif j.get('type') == 'prismatic':
            t = t @ homogeneous(translation=np.asarray(axis) * q)
        poses[link] = t
        return t

    for link in robot.findall('link'):
        pose(link.get('name'))
    return poses


def clearance(robot, cfg, q_deg, step=0.005, opening=None):
    values = {}
    for key, jcfg in cfg['joints'].items():
        values[jcfg['gazebo']['urdf_joint']] = mb.physical_to_urdf_rad(jcfg, q_deg[key])
    opening = cfg['startup']['gripper_opening_m'] if opening is None else opening
    for k in ('finger_a', 'finger_b'):
        values[cfg['gripper']['fingers'][k]['joint']] = mb.finger_position_for_opening(cfg, opening)
    poses = forward(robot, values)
    pts = {}
    for link in robot.findall('link'):
        p = link_collision_points(link, step)
        if len(p):
            pts[link.get('name')] = transform_points(poses[link.get('name')], p)
    fixed = [n for n in pts if n == 'Chasis' or n.endswith('_wheel_link') or n == 'base']
    # Profundidad máxima de cada eslabón móvil dentro de las cajas del chasis (0 = no entra)
    boxes = []
    for col in next(l for l in robot.findall('link') if l.get('name') == 'Chasis').findall('collision'):
        if col.find('geometry/box') is not None:
            t = poses['Chasis'] @ origin_matrix(col.find('origin'))
            boxes.append((np.linalg.inv(t), np.asarray(parse_floats(col.find('geometry/box').get('size'))) / 2))
    moving = ['Link_2', 'Link_3', 'Link_4', 'Link_5', 'finger_a', 'finger_b']
    parent = {j.find('child').get('link'): j.find('parent').get('link') for j in robot.findall('joint')}
    out = {}
    fixed_pts = np.vstack([pts[n] for n in fixed])
    tree = cKDTree(fixed_pts)
    for n in moving:
        d, _ = tree.query(pts[n])
        out[f'{n}-rover'] = float(d.min())
        out[f'{n}-piso'] = float(pts[n][:, 2].min())
        depth = 0.0
        for inv, half in boxes:
            local = transform_points(inv, pts[n])
            inside = np.all(np.abs(local) < half, axis=1)
            if inside.any():
                depth = max(depth, float(np.min(half - np.abs(local[inside]), axis=1).max()))
        out[f'{n}-dentro_chasis'] = -depth
    names = ['Link_1'] + moving
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if parent.get(a) == b or parent.get(b) == a or {a, b} == {'finger_a', 'finger_b'}:
                continue
            d, _ = cKDTree(pts[b]).query(pts[a])
            out[f'{a}-{b}'] = float(d.min())
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('q', nargs=5, type=float, help='J1..J5 en grados (convención de arm_manager)')
    ap.add_argument('--urdf', default=os.path.expanduser('~/.ros/rover_arm_sim/rover_arm.urdf'))
    ap.add_argument('--config', default=None)
    ap.add_argument('--step', type=float, default=0.005)
    args = ap.parse_args()
    cfg = mb.load_config(args.config)
    robot = ET.parse(args.urdf).getroot()
    q = dict(zip(['joint1', 'joint2', 'joint3', 'joint4', 'joint5'], args.q))
    for k, v in sorted(clearance(robot, cfg, q, args.step).items(), key=lambda kv: kv[1]):
        print(f'{k:22s} {v * 100:7.1f} cm')


if __name__ == '__main__':
    main()
