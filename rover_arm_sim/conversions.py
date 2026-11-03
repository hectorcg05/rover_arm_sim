"""Conversiones de los drivers reales, copiadas tal cual (mismas fórmulas, mismo redondeo).

Las pruebas de test/test_conversions.py comparan estas funciones contra el código real:
el C++ de drive_can_tank.cpp (compilado en un arnés) y el Python de joint_control.py.
"""

import struct


def float32(value):
    """drive_can_tank.cpp guarda los límites en variables `float` (cpp:70-78, 253-260)."""
    return struct.unpack('f', struct.pack('f', float(value)))[0]


def my_map(x, in_min, in_max, out_min, out_max):
    """drive_can_tank.cpp:88-90 y joint_control.py:19-21 (idénticas)."""
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def clamp(value, lo, hi):
    """joint_control.py:359-361."""
    return max(lo, min(hi, value))


class TalonConversion:
    """drive_can_tank (J1 y J4): rechazo fuera de rango, grados <-> ticks."""

    def __init__(self, deg_min, deg_max, ticks_min, ticks_max, invert, phys_error=0.0,
                 add_phys_error=False):
        self.deg_min = float32(deg_min)
        self.deg_max = float32(deg_max)
        self.ticks_min = float32(ticks_min)
        self.ticks_max = float32(ticks_max)
        self.invert = bool(invert)
        self.phys_error = float(phys_error)
        self.add_phys_error = bool(add_phys_error)

    @classmethod
    def from_cfg(cls, cfg):
        return cls(cfg['deg_min'], cfg['deg_max'], cfg['ticks_min'], cfg['ticks_max'],
                   cfg['invert'], cfg.get('phys_error', 0.0), cfg.get('add_phys_error', False))

    def command(self, msg_deg):
        """joint1_callback (cpp:368-379) / joint4_callback (cpp:382-393).

        Devuelve (aceptado, ángulo revisado, objetivo en ticks o None).
        """
        angle = msg_deg + self.phys_error if self.add_phys_error else msg_deg
        if angle < self.deg_min or angle > self.deg_max:
            return False, angle, None
        source = -angle if self.invert else angle
        return True, angle, my_map(source, self.deg_min, self.deg_max, self.ticks_min, self.ticks_max)

    def ticks_to_deg(self, ticks):
        """ticks_to_deg_j1 (cpp:92-95) / ticks_to_deg_j4 (cpp:97-100)."""
        value = my_map(ticks, self.ticks_min, self.ticks_max, self.deg_min, self.deg_max)
        return -value if self.invert else value

    def deg_to_ticks(self, deg):
        """Inversa de ticks_to_deg: dónde queda el encoder para un ángulo de driver dado."""
        source = -deg if self.invert else deg
        return my_map(source, self.deg_min, self.deg_max, self.ticks_min, self.ticks_max)

    def ticks_per_deg(self):
        return abs(self.ticks_max - self.ticks_min) / abs(self.deg_max - self.deg_min)


class EposConversion:
    """dual_epos_node (J2 y J3): recorte silencioso, grados <-> counts, home virtual."""

    def __init__(self, angle_min, angle_max, counts_min, counts_max):
        self.angle_min = float(angle_min)
        self.angle_max = float(angle_max)
        self.counts_min = float(counts_min)
        self.counts_max = float(counts_max)

    @classmethod
    def from_cfg(cls, cfg):
        return cls(cfg['angle_min'], cfg['angle_max'], cfg['counts_min'], cfg['counts_max'])

    def command(self, msg_deg):
        """cb_joint2 / cb_joint3 (joint_control.py:222-248): recorta y convierte; int() trunca."""
        angle = clamp(msg_deg, self.angle_min, self.angle_max)
        target = my_map(angle, self.angle_min, self.angle_max, self.counts_min, self.counts_max)
        return angle, int(target)

    def counts_to_deg(self, counts):
        """_counts_to_deg_j2 / _j3 (joint_control.py:200-208)."""
        return my_map(counts, self.counts_min, self.counts_max, self.angle_min, self.angle_max)

    def deg_to_counts(self, deg):
        return my_map(deg, self.angle_min, self.angle_max, self.counts_min, self.counts_max)

    def initial_counts(self, initial_angle):
        """Home virtual (joint_control.py:165-169)."""
        return int(my_map(initial_angle, self.angle_min, self.angle_max, self.counts_min, self.counts_max))

    def drive_limits(self, margin_counts):
        """Límite de posición del drive, 0x607D (joint_control.py:187-188)."""
        return (int(min(self.counts_min, self.counts_max)) - margin_counts,
                int(max(self.counts_min, self.counts_max)) + margin_counts)

    def counts_per_deg(self):
        return abs(self.counts_max - self.counts_min) / abs(self.angle_max - self.angle_min)
