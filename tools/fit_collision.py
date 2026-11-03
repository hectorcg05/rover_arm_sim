#!/usr/bin/env python3
"""Genera las mallas derivadas del gripper y las colisiones simplificadas del modelo.

Lee las mallas de CAD de mi_rover_description (sin modificarlas) y:
  1. Separa Link_5.STL en cuerpo + dos dedos (meshes/gripper_body.stl, finger_a.stl, finger_b.stl
     de este paquete). Los dedos quedan en el marco de Link_5, en la posición del CAD.
  2. Ajusta cajas de colisión (marco de cada link) a las mallas de los eslabones y del chasis, y
     cilindros a las llantas.
  3. Imprime el bloque YAML `collision:` para pegarlo en config/arm_sim.yaml.

Uso (desde la raíz del paquete):
    python3 tools/fit_collision.py --desc ../mi_rover_description --out meshes > /tmp/collision.yaml

Por qué cajas y no las mallas de CAD: las mallas tienen de 4 000 a 43 000 triángulos; con ellas como
colisión, Gazebo (DART) se vuelve lento e inestable con contactos (8 GB de RAM, criterio de
tiempo real). Las cajas siguen la forma de cada eslabón por tramos (error típico < 1 cm).
"""

import argparse
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from rover_arm_sim.geometry import origin_matrix, transform_points  # noqa: E402
from rover_arm_sim.stl_io import read_stl, sample_surface, write_stl  # noqa: E402

SAMPLE_STEP = 0.004          # m, muestreo de superficie para ajustar cajas
SLICE = 0.01                 # m, ancho de rebanada a lo largo del eje largo de cada eslabón
MERGE_TOLERANCE = 0.30       # unir rebanadas mientras el volumen crezca menos de 30 %
MAX_BOXES = 5

# Ejes largos de cada eslabón en su marco (ver tabla de rebanadas en documentacion_sim.md §4.2)
LINK_AXES = {'base': 0, 'Link_1': 2, 'Link_2': 1, 'Link_3': 1, 'Link_4': 1}

# Regiones de los dedos dentro de Link_5.STL (marco de Link_5): se encontraron rebanando la malla a
# lo largo de z (eje del gripper). Dedo A en y ≈ [-0.150, -0.122], dedo B en y ≈ [0.030, 0.060],
# los dos desde z ≈ 0.03 hasta la punta (z = 0.144). Abajo de z = 0.026 está el cuerpo con el riel.
FINGER_REGIONS = {
    'finger_a': {'y': (-0.155, -0.118), 'z_min': 0.026},
    'finger_b': {'y': (0.026, 0.064), 'z_min': 0.026},
}

# Regiones del chasis (marco Chasis, x adelante). La forma se sacó de un mapa de alturas de la malla
# de colisión de chasis.urdf: cubierta central, rieles laterales más bajos, travesaño trasero y
# patas delanteras/traseras que bajan hasta las llantas. Cada región se rellena de su z mínimo a su
# z máximo (envolvente "2.5D": el brazo llega siempre desde arriba, adelante o atrás).
CHASSIS_REGIONS = [
    ('deck', (-0.300, 0.350), (-0.250, 0.250)),
    ('rail_left', (-0.300, 0.350), (0.250, 0.382)),
    ('rail_right', (-0.300, 0.350), (-0.382, -0.250)),
    ('rear_bar', (-0.396, -0.300), (-0.150, 0.150)),
    ('rear_left', (-0.396, -0.300), (0.150, 0.382)),
    ('rear_right', (-0.396, -0.300), (-0.382, -0.150)),
    ('front_left', (0.350, 0.432), (0.150, 0.382)),
    ('front_right', (0.350, 0.432), (-0.382, -0.150)),
]


def box_of(points):
    lo, hi = points.min(0), points.max(0)
    return lo, hi


def _cross_area(points, axis):
    other = [i for i in range(3) if i != axis]
    return float(np.prod(np.ptp(points[:, other], axis=0)))


def fit_boxes_along(points, axis):
    lo, hi = points[:, axis].min(), points[:, axis].max()
    edges = np.arange(lo, hi + SLICE, SLICE)
    slices = []
    for a, b in zip(edges[:-1], edges[1:]):
        m = (points[:, axis] >= a) & (points[:, axis] <= b + 1e-9)
        if m.sum() >= 3:
            slices.append(points[m])
    # Fusión codiciosa por sección transversal: una rebanada se une al tramo actual si la sección
    # unida no crece más de MERGE_TOLERANCE respecto a cada una de las dos.
    segments = [slices[0]]
    for s in slices[1:]:
        merged = np.vstack([segments[-1], s])
        union = _cross_area(merged, axis)
        if union <= (1 + MERGE_TOLERANCE) * min(_cross_area(segments[-1], axis), _cross_area(s, axis)):
            segments[-1] = merged
        else:
            segments.append(s)
    while len(segments) > MAX_BOXES:
        # une el par contiguo cuya caja fusionada crezca menos
        best, cost = 0, None
        for i in range(len(segments) - 1):
            merged = np.vstack([segments[i], segments[i + 1]])
            c = np.prod(np.ptp(merged, axis=0)) - np.prod(np.ptp(segments[i], axis=0)) - \
                np.prod(np.ptp(segments[i + 1], axis=0))
            if cost is None or c < cost:
                best, cost = i, c
        segments[best:best + 2] = [np.vstack(segments[best:best + 2])]
    return [box_of(s) for s in segments]


def fmt_box(lo, hi):
    c = (lo + hi) / 2
    s = hi - lo
    return '{xyz: [%.4f, %.4f, %.4f], size: [%.4f, %.4f, %.4f]}' % (*c, *s)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--desc', required=True, help='carpeta de mi_rover_description')
    ap.add_argument('--out', required=True, help='carpeta donde escribir las mallas derivadas')
    args = ap.parse_args()
    mesh = os.path.join(args.desc, 'meshes')
    out = []

    # 1. Gripper: cuerpo + dedos
    tri = read_stl(os.path.join(mesh, 'Brazo', 'Link_5.STL'))
    cen = tri.mean(axis=1)
    used = np.zeros(len(tri), dtype=bool)
    finger_boxes = {}
    for name, reg in FINGER_REGIONS.items():
        m = (cen[:, 1] >= reg['y'][0]) & (cen[:, 1] <= reg['y'][1]) & (cen[:, 2] >= reg['z_min'])
        used |= m
        write_stl(os.path.join(args.out, name + '.stl'), tri[m])
        fp = sample_surface(tri[m], SAMPLE_STEP)
        # La caja empieza donde empieza el dedo (z_min); abajo está el riel, que ya cubre la caja
        # del cuerpo. Así la mordaza no roza la muñeca (Link_4) al girar J5.
        finger_boxes[name] = box_of(fp[fp[:, 2] >= reg['z_min']])
    body = tri[~used]
    write_stl(os.path.join(args.out, 'gripper_body.stl'), body)
    body_pts = sample_surface(body, SAMPLE_STEP)
    rail = body_pts[body_pts[:, 2] <= FINGER_REGIONS['finger_a']['z_min']]
    out.append('  # Generado por tools/fit_collision.py a partir de las mallas de mi_rover_description  # DEFAULT')
    out.append('  links:')
    for link, axis in LINK_AXES.items():
        pts = sample_surface(read_stl(os.path.join(mesh, 'Brazo', link + '.STL')), SAMPLE_STEP)
        boxes = fit_boxes_along(pts, axis)
        out.append(f'    {link}:')
        for lo, hi in boxes:
            out.append('      - ' + fmt_box(lo, hi))
    out.append('    Link_5:')
    out.append('      - ' + fmt_box(*box_of(rail)))
    out.append('  fingers:')
    for name, (lo, hi) in finger_boxes.items():
        out.append(f'    {name}: ' + fmt_box(lo, hi))

    # 2. Chasis (malla de colisión de chasis.urdf con su origen)
    chasis = ET.parse(os.path.join(args.desc, 'urdf', 'chasis.urdf')).getroot()
    col = chasis.find('link/collision')
    scale = float(col.find('geometry/mesh').get('scale').split()[0])
    pts = transform_points(origin_matrix(col.find('origin')),
                           sample_surface(read_stl(os.path.join(mesh, 'Chasis', 'Chasis.STL'), scale), SAMPLE_STEP))
    out.append('  chassis:')
    for name, (x0, x1), (y0, y1) in CHASSIS_REGIONS:
        m = (pts[:, 0] >= x0) & (pts[:, 0] <= x1) & (pts[:, 1] >= y0) & (pts[:, 1] <= y1)
        lo, hi = pts[m].min(0), pts[m].max(0)
        out.append(f'    - {{name: {name}, ' + fmt_box(lo, hi)[1:])

    # 3. Llantas: cilindro con el radio y ancho de la malla
    wpts = read_stl(os.path.join(mesh, 'Wheel', 'wheel.STL')).reshape(-1, 3)
    radius = float(np.max(np.linalg.norm(wpts[:, :2], axis=1)))
    width = float(np.ptp(wpts[:, 2]))
    out.append(f'  wheel: {{radius: {radius:.4f}, length: {width:.4f}}}')
    print('\n'.join(out))


if __name__ == '__main__':
    main()
