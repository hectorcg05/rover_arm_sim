"""Perfil de movimiento trapezoidal en línea (como Motion Magic del Talon y el PPM del EPOS).

El perfil se re-planifica en cada objetivo nuevo partiendo de la posición y velocidad actuales
(reporte §3.2 y S9). Se planifica de forma analítica, por tramos de aceleración constante, así que
los tiempos coinciden con la fórmula del trapecio. Trabaja en las unidades nativas del driver
(ticks o counts).
"""

import math


class TrapezoidProfile:
    def __init__(self, position, vmax, accel, decel=None):
        self.x = float(position)
        self.v = 0.0
        self.target = float(position)
        self.vmax = float(vmax)
        self.accel = float(accel)
        self.decel = float(decel if decel is not None else accel)
        self._segments = []   # [(duración, aceleración)]

    # ------------------------------------------------------------------ configuración
    def set_limits(self, vmax=None, accel=None, decel=None):
        changed = False
        for name, value in (('vmax', vmax), ('accel', accel), ('decel', decel)):
            if value is not None and float(value) != getattr(self, name):
                setattr(self, name, float(value))
                changed = True
        if changed:
            self._plan()

    def set_target(self, target):
        self.target = float(target)
        self._plan()

    def reset(self, position):
        """Detiene el perfil en `position` (por ejemplo, al limpiar una falla)."""
        self.x = float(position)
        self.v = 0.0
        self.target = float(position)
        self._segments = []

    @property
    def done(self):
        return not self._segments

    # ------------------------------------------------------------------ planificación
    def _plan(self):
        self._segments = []
        if self.vmax <= 0.0 or self.accel <= 0.0 or self.decel <= 0.0:
            self.v = 0.0
            return
        x, v = self.x, self.v
        segs = []
        dx = self.target - x
        if dx == 0.0 and v == 0.0:
            return
        s = math.copysign(1.0, dx) if dx != 0.0 else -math.copysign(1.0, v)
        vs = v * s   # velocidad en la dirección del objetivo
        if vs < 0.0 or vs * vs / (2.0 * self.decel) > abs(dx) + 1e-12:
            # Se aleja del objetivo o no alcanza a frenar antes: frena hasta 0 y replanifica.
            t = abs(v) / self.decel
            a = -math.copysign(self.decel, v)
            segs.append((t, a))
            x += v * t + 0.5 * a * t * t
            v = 0.0
            dx = self.target - x
            if dx == 0.0:
                self._segments = segs
                return
            s = math.copysign(1.0, dx)
            vs = 0.0
        d = abs(dx)
        if vs > self.vmax:
            # Viene más rápido que la velocidad máxima (bajaron el límite): baja a vmax.
            t = (vs - self.vmax) / self.decel
            segs.append((t, -s * self.decel))
            d -= (vs + self.vmax) / 2.0 * t
            vs = self.vmax
        vpeak = math.sqrt((2.0 * self.accel * self.decel * d + self.decel * vs * vs) /
                          (self.accel + self.decel))
        vpeak = max(vs, min(self.vmax, vpeak))
        t_acc = (vpeak - vs) / self.accel
        t_dec = vpeak / self.decel
        d_acc = (vs + vpeak) / 2.0 * t_acc
        d_dec = vpeak / 2.0 * t_dec
        t_cruise = max(0.0, (d - d_acc - d_dec) / vpeak) if vpeak > 0 else 0.0
        if t_acc > 0:
            segs.append((t_acc, s * self.accel))
        if t_cruise > 0:
            segs.append((t_cruise, 0.0))
        if t_dec > 0:
            segs.append((t_dec, -s * self.decel))
        self._segments = segs

    # ------------------------------------------------------------------ ejecución
    def step(self, dt):
        remaining = dt
        while remaining > 0.0 and self._segments:
            t, a = self._segments[0]
            h = min(t, remaining)
            self.x += self.v * h + 0.5 * a * h * h
            self.v += a * h
            remaining -= h
            if h >= t:
                self._segments.pop(0)
            else:
                self._segments[0] = (t - h, a)
        if not self._segments:
            # Fin del perfil: exactamente en el objetivo (quita el error de redondeo).
            self.x = self.target
            self.v = 0.0
        return self.x

    def remaining_time(self):
        return sum(t for t, _ in self._segments)
