"""Transformaciones homogéneas y cinemática directa del URDF de CAD (numpy)."""

import math

import numpy as np


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def homogeneous(rotation=None, translation=(0.0, 0.0, 0.0)):
    t = np.eye(4)
    if rotation is not None:
        t[:3, :3] = rotation
    t[:3, 3] = translation
    return t


def transform_points(t, points):
    return points @ t[:3, :3].T + t[:3, 3]


def parse_floats(text, default='0 0 0'):
    return [float(v) for v in (text if text is not None else default).split()]


def origin_matrix(element):
    """Matriz 4x4 de un elemento <origin> de URDF (o identidad si no hay)."""
    if element is None:
        return np.eye(4)
    xyz = parse_floats(element.get('xyz'))
    rpy = parse_floats(element.get('rpy'))
    return homogeneous(rpy_to_matrix(*rpy), xyz)
