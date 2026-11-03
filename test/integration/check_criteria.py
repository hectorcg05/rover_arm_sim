"""Verificación de los criterios de la v1 contra una simulación EN MARCHA (Gazebo sin ventana).

Uso (la sim ya lanzada en el mismo ROS_DOMAIN_ID):
    python3 check_criteria.py c1 --log <log del launch>
    python3 check_criteria.py c3 | c4 | c5 | c8 | c10
    python3 check_criteria.py speed          # mide cuánto tarda J2 en 30° (criterio 9, con dos YAML)
Los criterios 6 y 7 están en check_fault.py y check_grasp.py. run_all.sh los corre todos.
Cada prueba imprime "RESULTADO <criterio> OK|FALLA" y los valores medidos.
"""

import argparse
import copy
import math
import os
import re
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

from simprobe import Probe, move_joints

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'tools'))


def result(name, ok, **values):
    print(f'RESULTADO {name} {"OK" if ok else "FALLA"} ' + ' '.join(f'{k}={v}' for k, v in values.items()))
    return ok


def resync(p):
    """idle -> jog: arm_manager vuelve a leer los encoders (arm_manager.py:323)."""
    p.publish('/arm_mode/set', String, 'idle')
    time.sleep(0.4)
    p.publish('/arm_mode/set', String, 'jog')
    time.sleep(0.6)


class JointStates:
    def __init__(self, node):
        self.pos = {}
        node.create_subscription(JointState, '/joint_states', self._cb, 10)

    def _cb(self, m):
        self.pos.update(dict(zip(m.name, m.position)))


def home_pose(p):
    """Regresa a la postura de arranque 0/170/−146/0 por etapas seguras y resincroniza."""
    if abs(p.fb[2] - 170.0) > 0.5 or abs(p.fb[3] + 146.0) > 0.5 or abs(p.fb[1]) > 0.5 or abs(p.fb[4]) > 0.5:
        for stage in ({4: 0.0}, {3: -90.0}, {2: 90.0}, {1: 0.0}, {3: -146.0}, {2: 170.0}):
            move_joints(p, stage, timeout=20)
    p.publish('/arm_teleop/joint5', Float64, 0.0)
    time.sleep(2.0)
    resync(p)


def settle(p, timeout=8.0):
    """Espera a que el brazo deje de moverse (el brazo real sigue moviéndose al soltar, C7/C9)."""
    t0 = time.monotonic()
    last = p.tip()
    while time.monotonic() - t0 < timeout:
        p.joy()
        time.sleep(0.5)
        now = p.tip()
        if max(abs(a - b) for a, b in zip(now, last)) < 1e-4:
            return now
        last = now
    return last


# ---------------------------------------------------------------------------------- criterio 1
def c1(args):
    text = open(args.log, errors='replace').read()
    m = re.search(r'Initialized from encoders: q=\(([-\d.]+), ([-\d.]+), ([-\d.]+), ([-\d.]+)\)', text)
    timeout = 'Timeout! Missing' in text
    q = [float(v) for v in m.groups()] if m else None
    ok = bool(m) and not timeout and all(abs(a - b) < 0.05 for a, b in zip(q, (0, 170, -146, 0)))
    return result('c1', ok, q=q, timeout=timeout)


# ---------------------------------------------------------------------------------- criterio 3
def c3(_args):
    p = Probe()
    js = JointStates(p.node)
    p.wait_ready()
    home_pose(p)
    ok = True
    x0 = p.tip()
    p.hold_joy(2.5, axes=(0.0, 1.0, 0.0))                       # palanca adelante
    p.wait(1.0)
    x1 = p.tip()
    fwd = x1[0] - x0[0]
    ok &= fwd > 0.05
    print(f'palanca adelante 2.5 s: la punta avanzó {fwd * 100:.1f} cm en x (z {100 * (x1[2] - x0[2]):+.1f} cm)')
    p.hold_joy(2.5, axes=(0.0, 1.0, 0.0), buttons=(0,))         # gatillo + adelante
    p.wait(1.0)
    x2 = p.tip()
    up = x2[2] - x1[2]
    ok &= up > 0.05 and abs(x2[0] - x1[0]) < 0.02
    print(f'gatillo + adelante 2.5 s: subió {up * 100:.1f} cm (x {100 * (x2[0] - x1[0]):+.1f} cm)')
    j5 = js.pos.get('Joint_5', 0.0)
    p.hold_joy(1.0, buttons=(5,))                               # botón 6 = roll +
    p.wait(1.0)
    roll_plus = math.degrees(js.pos['Joint_5'] - j5)
    p.hold_joy(1.0, buttons=(6,))                               # botón 7 = roll −
    p.wait(1.5)
    roll_minus = math.degrees(js.pos['Joint_5'] - j5) - roll_plus
    ok &= roll_plus > 10 and roll_minus < -10
    print(f'botón 6 (1 s): J5 {roll_plus:+.1f}°; botón 7 (1 s): J5 {roll_minus:+.1f}°')
    resync(p)
    pitch0 = math.degrees(p.tip()[3])
    p.hold_joy(1.0, buttons=(10,))                              # botón 11 = pitch +
    pitch1 = math.degrees(settle(p)[3])                         # J4 va atrás del pitch pedido (C9)
    p.hold_joy(1.0, buttons=(9,))                               # botón 10 = pitch −
    pitch2 = math.degrees(settle(p)[3])
    ok &= pitch1 - pitch0 > 10 and pitch2 - pitch1 < -10
    print(f'botón 11 (1 s): pitch {pitch1 - pitch0:+.1f}°; botón 10 (1 s): pitch {pitch2 - pitch1:+.1f}°')
    g0 = p.status['gripper']['opening_m']
    p.hold_joy(1.5, buttons=(4,))                               # botón 5 = cerrar
    p.wait(0.5)
    g1 = p.status['gripper']['opening_m']
    p.hold_joy(1.5, buttons=(3,))                               # botón 4 = abrir
    p.wait(0.5)
    g2 = p.status['gripper']['opening_m']
    ok &= g1 < g0 - 0.04 and g2 > g1 + 0.04
    print(f'botón 5 (1.5 s): apertura {100 * g0:.1f} → {100 * g1:.1f} cm; botón 4 (1.5 s): → {100 * g2:.1f} cm')
    p.close()
    return result('c3', ok, avance_cm=round(fwd * 100, 1), subida_cm=round(up * 100, 1))


# ---------------------------------------------------------------------------------- criterio 4
def c4(_args):
    p = Probe()
    p.wait_ready()
    ok = True
    for j, bad, lim in ((1, 95.0, '[-85.0, 90.0]'), (1, -90.0, '[-85.0, 90.0]'), (4, 155.0, '[-150.0, 150.0]')):
        before = p.fb[j]
        t0 = time.monotonic()
        p.publish(f'/arm_teleop/joint{j}', Float64, bad)
        time.sleep(1.5)
        warns = [l[3] for l in p.logs_since(t0, names=['arm_drive_ctre'], min_level=30)]
        expected = f'J{j} angle {bad:.1f} out of range {lim}'
        good = expected in warns and abs(p.fb[j] - before) < 0.1
        ok &= good
        print(f'J{j} ← {bad}: se queda en {p.fb[j]:.2f}° (antes {before:.2f}°); WARN = {warns}')
    for j, bad, clamp in ((2, 185.0, 170.0), (3, -175.0, -160.0)):
        t0 = time.monotonic()
        p.publish(f'/arm_teleop/joint{j}', Float64, bad)
        move_joints(p, {j: clamp}, timeout=8)
        time.sleep(0.5)
        warns = [l[3] for l in p.logs_since(t0, names=['dual_epos_node'], min_level=30)]
        good = abs(p.fb[j] - clamp) < 0.05 and not warns and not p.status.get('faults')
        ok &= good
        print(f'J{j} ← {bad}: recortado en silencio a {p.fb[j]:.2f}° (WARN: {warns})')
    move_joints(p, {3: -146.0}, timeout=8)
    resync(p)
    p.close()
    return result('c4', ok)


# ---------------------------------------------------------------------------------- criterio 5
def predict(cfg, commands, t_release, t_end, start):
    """Re-simula los drivers del YAML con las mismas órdenes que mandó arm_manager (seguimiento
    perfecto de Gazebo) y devuelve la punta predicha después de soltar la palanca."""
    from fakes import FakeClock, FakeNode
    from rover_arm_sim.drivers import model_class
    from rover_arm_sim.drivers.groups import EposGroup, TalonGroup
    clock = FakeClock()
    models, nodes = {}, {}
    for key in ('joint1', 'joint2', 'joint3', 'joint4'):
        jcfg = cfg['joints'][key]
        nodes[key] = FakeNode()
        models[key] = model_class(jcfg['driver_model'])(key, jcfg, nodes[key], clock)
        gk = cfg['driver_nodes'][jcfg['driver_node']]['kind']
        grp = {'talon_group': TalonGroup, 'epos_group': EposGroup}[gk](jcfg['driver_node'],
                                                                       cfg['driver_nodes'][jcfg['driver_node']],
                                                                       nodes[key], clock)
        grp.add(models[key])
        models[key].start(start[key])
    pos = dict(start)
    t = commands[0][0] - 0.5
    clock.t = t
    out, i = [], 0
    while t < t_end:
        t += 0.01
        clock.t = t
        while i < len(commands) and commands[i][0] <= t:
            _, j, v = commands[i]
            nodes[f'joint{j}'].send(f'/arm_teleop/joint{j}', v)
            i += 1
        for k, m in models.items():
            m.update(t, 0.01, pos[k])
            pos[k] = m.gazebo_command(pos[k])
        out.append((t, dict(pos)))
    return out


def c5(args):
    from rover_arm_sim import model_builder as mb
    cfg = mb.load_config(args.config)
    p = Probe()
    p.wait_ready()
    home_pose(p)
    cmds = []
    for j in (1, 2, 3, 4):
        p.node.create_subscription(Float64, f'/arm_teleop/joint{j}',
                                   lambda m, j=j: cmds.append((time.monotonic(), j, m.data)), 10)
    time.sleep(0.5)
    start = {f'joint{j}': p.fb[j] for j in (1, 2, 3, 4)}
    rec = []
    p.hold_joy(3.0, axes=(0.0, 1.0, 0.0), record=rec)
    t_rel = time.monotonic()
    p.hold_joy(3.0, record=rec)
    pose_at_release = next(r[2] for r in rec if r[0] >= t_rel)
    after = [(t - t_rel, tip) for t, tip, _ in rec if t >= t_rel]
    tip_rel = after[0][1]
    final = after[-1][1]
    drift = math.dist(tip_rel[:3], final[:3])
    moving_until = max((t for t, tip in after if math.dist(tip[:3], final[:3]) > 0.001), default=0.0)
    pred = predict(cfg, sorted(cmds), t_rel, t_rel + 3.0, start)
    k = p.kin
    ptips = [(t - t_rel, k.forward(s['joint1'], s['joint2'], s['joint3'], s['joint4'], 0.0)) for t, s in pred if t >= t_rel]
    pf = ptips[-1][1]
    p_rel = ptips[0][1]
    p_drift = math.dist((p_rel['x'], p_rel['y'], p_rel['z']), (pf['x'], pf['y'], pf['z']))
    p_until = max((t for t, f in ptips if math.dist((f['x'], f['y'], f['z']), (pf['x'], pf['y'], pf['z'])) > 0.001),
                  default=0.0)
    print(f'al soltar, arm_manager ya pedía x={pose_at_release[0]:.3f}; la punta medida estaba en x={tip_rel[0]:.3f}')
    print(f'medido: siguió {drift * 100:.2f} cm durante {moving_until:.2f} s')
    print(f'predicho con las velocidades del YAML (J2 {cfg["joints"]["joint2"]["epos_ppm"]["speed_deg_s"]} °/s, '
          f'J3 {cfg["joints"]["joint3"]["epos_ppm"]["speed_deg_s"]} °/s): {p_drift * 100:.2f} cm durante {p_until:.2f} s')
    ok = drift > 0.002 and abs(drift - p_drift) < 0.004 + 0.25 * p_drift and abs(moving_until - p_until) < 0.15
    p.close()
    return result('c5', ok, medido_cm=round(drift * 100, 2), predicho_cm=round(p_drift * 100, 2),
                  medido_s=round(moving_until, 2), predicho_s=round(p_until, 2))


# ---------------------------------------------------------------------------------- criterio 8
def c8(args):
    from check_clearance import clearance
    from rover_arm_sim import model_builder as mb
    cfg = mb.load_config(args.config)
    robot = ET.parse(os.path.expanduser('~/.ros/rover_arm_sim/rover_arm.urdf')).getroot()
    p = Probe()
    p.wait_ready()
    home_pose(p)

    def measured_clearance():
        q = {f'joint{j}': p.fb[j] for j in (1, 2, 3, 4)}
        q['joint5'] = 0.0
        return clearance(robot, cfg, q, step=0.005)

    ok = True
    # a) Chasis: gripper hacia la cubierta (sin chasis, las mordazas entrarían 5.8 cm)
    target = {3: -160.0, 4: -90.0}
    d_free = clearance(robot, cfg, {'joint1': 0.0, 'joint2': 170.0, 'joint3': -160.0, 'joint4': -90.0,
                                    'joint5': 0.0}, step=0.005)
    free_pen = min(v for k, v in d_free.items() if k.endswith('dentro_chasis'))
    move_joints(p, {3: -160.0}, timeout=6)
    move_joints(p, {4: -90.0}, timeout=6)
    time.sleep(2.0)
    d = measured_clearance()
    pen = min(v for k, v in d.items() if k.endswith('dentro_chasis'))
    stuck = {j: round(p.fb[j], 1) for j in target}
    print(f'chasis: sin choque la mordaza entraría {-free_pen * 100:.1f} cm; con la sim se detuvo en {stuck}, '
          f'penetración {-pen * 100:.2f} cm; estado {p.status.get("faults")}, atascados {p.status.get("stalled")}')
    ok &= pen > -0.01
    move_joints(p, {4: 0.0}, timeout=8)
    subprocess.run(['ros2', 'service', 'call', '/rover_arm_sim/reset_driver', 'std_srvs/srv/Trigger'],
                   capture_output=True, timeout=20)
    move_joints(p, {3: -146.0}, timeout=8)
    # b) Piso: brazo estirado hacia abajo en una zona libre
    for stage in ({3: -90.0, 4: 0.0}, {2: 90.0}, {1: 60.0}, {2: 20.0, 3: -90.0, 4: 0.0}):
        move_joints(p, stage, timeout=15)
    move_joints(p, {2: 0.0}, timeout=6)
    time.sleep(1.5)
    d = measured_clearance()
    floor = min(v for k, v in d.items() if k.endswith('-piso'))
    free = clearance(robot, cfg, {'joint1': 60.0, 'joint2': 0.0, 'joint3': -90.0, 'joint4': 0.0, 'joint5': 0.0},
                     step=0.005)
    free_floor = min(v for k, v in free.items() if k.endswith('-piso'))
    print(f'piso: sin piso el gripper bajaría a z={free_floor * 100:.1f} cm; con la sim quedó en z={floor * 100:.2f} cm '
          f'(J2 {p.fb[2]:.1f}°), fallas {p.status.get("faults")}')
    ok &= floor > -0.01
    subprocess.run(['ros2', 'service', 'call', '/rover_arm_sim/reset_driver', 'std_srvs/srv/Trigger'],
                   capture_output=True, timeout=20)
    for stage in ({2: 90.0}, {1: 0.0}, {3: -146.0, 4: 0.0}, {2: 170.0}):
        move_joints(p, stage, timeout=15)
    resync(p)
    p.close()
    return result('c8', ok, penetracion_chasis_cm=round(-pen * 100, 2), punto_mas_bajo_cm=round(floor * 100, 2))


# ---------------------------------------------------------------------------------- criterio 9
def speed(_args):
    p = Probe()
    p.wait_ready()
    move_joints(p, {2: 170.0}, tol=0.01, timeout=20)
    time.sleep(1.0)
    p.publish('/arm_teleop/joint2', Float64, 140.0)
    t0 = time.monotonic()
    start = None
    while time.monotonic() - t0 < 15:
        if start is None and abs(p.fb[2] - 170.0) > 0.01:
            start = time.monotonic()
        if abs(p.fb[2] - 140.0) < 0.01:
            break
        time.sleep(0.005)
    t = time.monotonic() - (start or t0)
    move_joints(p, {2: 170.0}, timeout=15)
    p.close()
    print(f'J2 170° → 140°: {t:.2f} s')
    return result('speed', True, segundos=round(t, 2))


# ---------------------------------------------------------------------------------- criterio 10
def c10(args):
    out = subprocess.run(['ign', 'topic', '-e', '-t', '/world/practica_brazo/stats', '-n', str(args.samples)],
                         capture_output=True, text=True, timeout=args.samples / 4 + 30).stdout
    rtf = [float(v) for v in re.findall(r'real_time_factor:\s*([\d.]+)', out)]
    mean = sum(rtf) / len(rtf) if rtf else 0.0
    print(f'{len(rtf)} muestras de /stats: RTF medio {mean:.3f}, mínimo {min(rtf):.3f}, máximo {max(rtf):.3f}')
    return result('c10', mean > 0.95, rtf_medio=round(mean, 3), rtf_min=round(min(rtf), 3))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('criterion')
    ap.add_argument('--log')
    ap.add_argument('--config', default=None)
    ap.add_argument('--samples', type=int, default=100)
    args = ap.parse_args()
    ok = globals()[args.criterion](args)
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
