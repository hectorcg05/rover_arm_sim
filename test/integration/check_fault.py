"""Criterio 6: chocar J2 contra el piso produce la falla del EPOS; "Reiniciar driver" la limpia.

1. Lleva las mordazas a unos 7 cm del piso en una zona libre.
2. Manda J2 más abajo: el brazo choca con el piso, J2 no puede seguir su perfil y su EPOS entra en
   falla 0x8611 (mismo log que dual_epos_node).
3. Comprueba que J2 ya no responde a órdenes.
4. Llama a /rover_arm_sim/reset_driver (lo mismo que el botón "Reiniciar driver" de la ventana) y
   comprueba que J2 vuelve a responder y que la posición no se redeclaró.
"""
import subprocess
import sys
import time

from simprobe import Probe, move_joints

SAFE = [{3: -90.0, 4: 0.0}, {2: 90.0}, {1: 60.0}]
NEAR_FLOOR = {1: 60.0, 2: 36.0, 3: -100.0, 4: -20.0}   # mordazas a ~7 cm del piso (tools/check_clearance.py)


def main():
    p = Probe()
    p.wait_ready()
    for stage in SAFE + [NEAR_FLOOR]:
        print('mover', stage, '->', move_joints(p, stage, timeout=20)[0], 'fallas', p.status.get('faults'))
    t0 = time.monotonic()
    before = p.fb[2]
    print('J2 contra el piso: orden 15°')
    move_joints(p, {2: 15.0}, timeout=6)
    time.sleep(1.5)   # el monitor de fallas del real revisa a 1 Hz
    faults = p.status.get('faults', [])
    stuck = p.fb[2]
    errs = [l[3] for l in p.logs_since(t0, names=['dual_epos_node'], min_level=40)]
    print('J2 se detuvo en', round(stuck, 2), '(partió de', round(before, 2), ') fallas', faults)
    print('log de dual_epos_node:', errs)
    # ¿Responde en falla?
    move_joints(p, {2: 45.0}, timeout=3)
    print('en falla, orden 45° -> J2 =', round(p.fb[2], 2))
    responds_in_fault = abs(p.fb[2] - stuck) > 0.5
    out = subprocess.run(['ros2', 'service', 'call', '/rover_arm_sim/reset_driver', 'std_srvs/srv/Trigger'],
                         capture_output=True, text=True, timeout=20).stdout
    print('reset:', out.strip().splitlines()[-1])
    time.sleep(1.5)
    infos = [l[3] for l in p.logs_since(t0, names=['dual_epos_node'], min_level=20) if 'resuelta' in l[3]]
    after_reset = p.fb[2]
    move_joints(p, {2: 45.0}, timeout=8)
    print('tras reiniciar: J2 en', round(after_reset, 2), '-> orden 45° ->', round(p.fb[2], 2),
          'fallas', p.status.get('faults'), 'log:', infos)
    ok = ('Joint2' in faults and errs and 'Following error' in errs[0] and not responds_in_fault and
          abs(after_reset - stuck) < 0.5 and abs(p.fb[2] - 45.0) < 0.5 and infos and not p.status.get('faults'))
    print('RESULTADO c6', 'OK' if ok else 'FALLA')
    p.close()
    return ok


if __name__ == '__main__':
    sys.exit(0 if main() else 1)
