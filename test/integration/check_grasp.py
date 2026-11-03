"""Criterio 7: el gripper levanta y transporta un objeto sin soltarlo.

Mueve el brazo con ángulos directos (como la prueba P3) y el gripper con el botón 5 del joystick a
través del teleop real (/joy → simple_drive_arm → /arm_teleop/gripper). Uso:
    python3 grasp_test.py caja_5kg|contenedor_asa
"""
import math
import sys
import time

from simprobe import Probe, move_joints, object_pose

# Posturas calculadas con la IK real (documentacion_sim.md §5, tabla de agarre)
PLANS = {
    'caja_5kg': {'pre': {1: 0.5, 2: 52.0, 3: -105.0, 4: -37.0},
                 'grasp': {1: 0.5, 2: 33.1, 3: -96.0, 4: -27.1}},
    'contenedor_asa': {'pre': {1: 40.1, 2: 42.2, 3: -67.2, 4: -65.0},
                       'grasp': {1: 40.1, 2: 33.9, 3: -69.5, 4: -54.4}},
}
SAFE = [{3: -90.0, 4: 0.0}, {2: 90.0}]   # salir de la postura de arranque sin rozar nada


def hold_close(p, seconds, rec=None):
    """Mantiene el botón 5 (índice 4 = cerrar) mientras pasa `seconds`."""
    p.hold_joy(seconds, buttons=(4,))


def run(name):
    p = Probe()
    p.wait_ready()
    plan = PLANS[name]
    start = object_pose(name)
    print('objeto al inicio', start)
    for stage in SAFE + [plan['pre'], plan['grasp']]:
        print('mover', stage, '->', move_joints(p, stage, timeout=20)[0], 'fallas', p.status.get('faults'))
    # Cerrar: botón 5 mantenido; el teleop publica -1 en /arm_teleop/gripper a 10 Hz
    p.hold_joy(3.0, buttons=(4,))
    g = p.status.get('gripper', {})
    print('gripper tras cerrar', {k: g.get(k) for k in ('opening_m', 'holding', 'blocked', 'touching')})
    # Levantar y transportar sin soltar el botón
    from std_msgs.msg import Float64
    t0 = time.monotonic()
    for j, v in plan['pre'].items():
        p.publish(f'/arm_teleop/joint{j}', Float64, v)
    p.hold_joy(4.0, buttons=(4,))
    lifted = object_pose(name)
    p.publish('/arm_teleop/joint1', Float64, plan['pre'][1] - 35.0)
    p.hold_joy(5.0, buttons=(4,))
    moved = object_pose(name)
    g = p.status.get('gripper', {})
    print('levantado', lifted, 'transportado', moved, 'sujeto', g.get('holding'), 'fallas', p.status.get('faults'))
    ok = (g.get('holding') == name and lifted[2] - start[2] > 0.08 and
          math.hypot(moved[0] - lifted[0], moved[1] - lifted[1]) > 0.2)
    # Soltar: deja de apretar el botón -> el objeto se suelta y cae
    p.hold_joy(2.0)
    print('soltado, sujeto =', repr(p.status.get('gripper', {}).get('holding')), 'objeto', object_pose(name))
    print(f'RESULTADO c7_{name}', 'OK' if ok else 'FALLA')
    p.close()
    return ok


if __name__ == '__main__':
    sys.exit(0 if run(sys.argv[1]) else 1)
