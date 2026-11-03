"""Modelo `dc_open_loop`: el gripper (motor DC de 12 V con controlador externo, reporte N2).

Escucha /arm_teleop/gripper (Float64 −1/0/+1, drive_arm_teleop.py:135-143). El valor es el
porcentaje de voltaje: la apertura cambia a valor · (carrera / tiempo de carrera completa), la mitad
en cada mordaza. Las mordazas se detienen en 0 y en la carrera máxima (S12) y cuando chocan con
algo; detenidas, siguen empujando con la fuerza máxima del modelo (squeeze_margin_m de ventaja).

Sujetar: mientras el operador cierra y LAS DOS mordazas tocan el mismo objeto, el emulador le pide
al plugin GraspWeld de Gazebo que lo sujete (documentacion_sim.md, "Gripper"). Con
hold_requires_button, lo suelta en cuanto el operador deja de cerrar.
"""

from std_msgs.msg import Float64


class DcGripper:
    def __init__(self, cfg, node, clock, initial_opening):
        self.cfg = cfg
        self.node = node
        self.clock = clock
        f = cfg['fingers']
        self.stroke = float(cfg['stroke_m'])
        self.speed = self.stroke / float(cfg['full_stroke_time_s'])   # m/s de apertura total
        self.open_sign = float(cfg['open_sign'])
        self.cad_opening = float(f['cad_opening_m'])
        self.squeeze = float(cfg['squeeze_margin_m'])
        self.lo, self.hi = self.finger_position(0.0), self.finger_position(self.stroke)
        p = self.finger_position(max(0.0, min(self.stroke, float(initial_opening))))
        self.targets = [p, p]
        self.cmd = 0.0
        self.blocked = [False, False]
        self.at_end_since = None
        self.at_end = False
        self.holding = ''
        self.touching = {}
        node.create_subscription(Float64, cfg['command_topic'], self._on_command, 10)

    def finger_position(self, opening):
        """Desplazamiento de cada mordaza (sobre su eje, hacia afuera) para una apertura total."""
        return (opening - self.cad_opening) / 2.0

    def opening_from(self, pos_a, pos_b):
        return self.cad_opening + pos_a + pos_b

    def finger_limits(self):
        return self.lo, self.hi

    def _on_command(self, msg):
        self.cmd = max(-1.0, min(1.0, msg.data))

    @property
    def closing(self):
        return self.cmd * self.open_sign < 0.0

    def update(self, now, dt, measured):
        """measured: posiciones medidas de las dos mordazas (m)."""
        velocity = self.cmd * self.open_sign * self.speed / 2.0    # m/s por mordaza (+ abre)
        for i, m in enumerate(measured):
            if self.holding or velocity == 0.0:
                # Sujetando, o motor apagado: las mordazas se quedan donde están.
                self.targets[i] = max(self.lo, min(self.hi, m))
                self.blocked[i] = bool(self.holding)
                continue
            t = self.targets[i] + velocity * dt
            # La orden no se adelanta más de squeeze_margin_m a la mordaza real: si algo la detiene,
            # el motor empuja con su fuerza máxima en vez de "acumular" recorrido.
            t = max(m - self.squeeze, min(m + self.squeeze, t))
            self.targets[i] = max(self.lo, min(self.hi, t))
            self.blocked[i] = abs(self.targets[i] - m) >= 0.9 * self.squeeze
        end = velocity != 0.0 and all(t <= self.lo or t >= self.hi for t in self.targets)
        if end:
            self.at_end_since = self.at_end_since if self.at_end_since is not None else now
            self.at_end = now - self.at_end_since >= float(self.cfg['end_stop_warning_time_s'])
        else:
            self.at_end_since = None
            self.at_end = False

    def finger_commands(self):
        return tuple(self.targets)

    def grasp_request(self):
        """'hold' mientras el operador cierra; si no, 'release' (con hold_requires_button)."""
        if self.closing:
            return 'hold'
        if self.holding and not self.cfg.get('hold_requires_button', True) and not self.cmd:
            return 'hold'
        return 'release'

    def status(self, measured):
        return {'opening_m': round(self.opening_from(*measured), 4), 'command': self.cmd,
                'holding': self.holding, 'blocked': list(self.blocked), 'at_end_stop': self.at_end,
                'touching': self.touching}
