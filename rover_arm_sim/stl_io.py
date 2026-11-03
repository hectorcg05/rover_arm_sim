"""Lectura y escritura de STL binario, y muestreo de superficie (solo numpy)."""

import struct

import numpy as np

_STL_DTYPE = np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')])


def read_stl(path, scale=1.0):
    """Devuelve los triángulos de un STL binario como arreglo (N, 3, 3)."""
    with open(path, 'rb') as f:
        data = f.read()
    count = struct.unpack('<I', data[80:84])[0]
    tri = np.frombuffer(data[84:84 + count * 50], dtype=_STL_DTYPE)
    return tri['v'].reshape(-1, 3, 3).astype(float) * scale


def write_stl(path, triangles, header=b'rover_arm_sim'):
    """Escribe triángulos (N, 3, 3) como STL binario."""
    triangles = np.asarray(triangles, dtype=float)
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    norm = np.linalg.norm(normals, axis=1, keepdims=True)
    norm[norm == 0] = 1.0
    rec = np.zeros(len(triangles), dtype=_STL_DTYPE)
    rec['n'] = normals / norm
    rec['v'] = triangles.reshape(-1, 9)
    with open(path, 'wb') as f:
        f.write(header.ljust(80, b' ')[:80])
        f.write(struct.pack('<I', len(triangles)))
        f.write(rec.tobytes())


def sample_surface(triangles, step):
    """Puntos sobre la superficie de los triángulos, separados ~`step` (incluye vértices)."""
    pts = [triangles.reshape(-1, 3)]
    for t in triangles:
        e1, e2 = t[1] - t[0], t[2] - t[0]
        m = int(max(np.linalg.norm(e1), np.linalg.norm(e2)) / step)
        if m < 1:
            continue
        u, v = np.meshgrid(np.linspace(0, 1, m + 1), np.linspace(0, 1, m + 1))
        k = (u + v) <= 1
        pts.append(t[0] + np.outer(u[k], e1) + np.outer(v[k], e2))
    return np.vstack(pts)
