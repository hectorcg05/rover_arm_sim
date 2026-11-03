"""Modelo `epos_ppm`: emula J2/J3 de dual_epos_node (EPOS4 en Profile Position Mode)."""

import collections

from std_msgs.msg import Float64

from ..conversions import EposConversion
from ..profiles import TrapezoidProfile
from .base import DriverModel
from .groups import EPOS_ERROR_NAMES, EPOS_ERROR_REGISTER_FLAGS


class EposPPM(DriverModel):
    def __init__(self, name, cfg, node, clock):
        super().__init__(name, cfg, node, clock)
        c = cfg['epos_ppm']
        self.c = c
        self.conv = EposConversion.from_cfg(c)
        cpd = self.conv.counts_per_deg()
        # La unidad del perfil del EPOS no se sabe (reporte §3.2): se fija con la velocidad física
        # medida (P3) para los valores de perfil del launch real.
        self.counts_s_per_vel_unit = float(c['speed_deg_s']) * cpd / float(c['profile_velocity'])
        self.counts_s2_per_acc_unit = float(c['accel_deg_s2']) * cpd / float(c['profile_acceleration'])
        self.sync_vel = int(c['profile_velocity'])                                # joint_control.py:88
        self.max_vel = int(c['profile_velocity']) * int(c['max_velocity_factor'])   # 0x607F
        self.max_acc = int(c['profile_acceleration']) * int(c['max_acceleration_factor'])  # 0x60C5
        self.mode = c['setpoint_mode']
        if self.mode not in ('immediate', 'buffered'):
            raise ValueError(f'{name}: setpoint_mode debe ser immediate o buffered, no {self.mode!r}')
        self.latency = float(c['command_latency_ms']) / 1000.0
        self.window_counts = float(c['following_error_window_counts'])
        self.tracking_window = float(c['sim_tracking_window_s'])
        self.home_shift = 0.0
        self.profile = None
        self.pending = collections.deque()        # (t_aplica, counts, vel, acc, dec)
        self.buffered = None
        self.history = collections.deque()        # (t, orden física en grados)
        self.fault = None
        self.fault_history = []
        self.hold_phys = None
        self.measured_counts = None
        self.last_error_counts = 0.0
        self.pub_deg = node.create_publisher(Float64, cfg['feedback_deg_topic'], 10)
        self.pub_raw = node.create_publisher(Float64, cfg['feedback_raw_topic'], 10)
        node.create_subscription(Float64, cfg['command_topic'], self._on_command, 10)
        node.create_subscription(Float64, cfg['sync_velocity_topic'], self._on_sync, 10)

    @staticmethod
    def driver_limits_deg(cfg):
        c = cfg['epos_ppm']
        conv = EposConversion.from_cfg(c)
        lo, hi = conv.drive_limits(int(c['drive_limit_margin_counts']))
        a, b = conv.counts_to_deg(lo), conv.counts_to_deg(hi)
        return min(a, b), max(a, b)

    # ------------------------------------------------------------------------------------------
    def _counts_from_physical(self, phys_deg):
        """Posición del encoder (0x6064, entero) para un ángulo físico, con el home virtual."""
        return float(round(self.conv.deg_to_counts(self.physical.to_driver(phys_deg)) + self.home_shift))

    def _physical_from_counts(self, counts):
        return self.physical.to_physical(self.conv.counts_to_deg(counts - self.home_shift))

    def start(self, measured_phys):
        # Home virtual (joint_control.py:164-180, epos_controller.py:201-215): "aquí es initial_angle".
        initial_angle = float(self.c['initial_angle'])
        initial_counts = self.conv.initial_counts(initial_angle)
        self.home_shift = 0.0
        raw = self.conv.deg_to_counts(self.physical.to_driver(measured_phys))
        self.home_shift = initial_counts - raw
        self.measured_counts = float(initial_counts)
        self.node.get_logger().info(
            f'{self.log_name}: home establecido en {initial_angle}° ({initial_counts} counts)')
        self.profile = TrapezoidProfile(initial_counts, self._vel(self.sync_vel),
                                        self._acc(int(self.c['profile_acceleration'])),
                                        self._acc(int(self.c['profile_deceleration'])))
        self.node.get_logger().info(f'{self.log_name} (ID {self.c["node_id"]}): PPM trapezoidal ready')
        super().start(measured_phys)

    def _vel(self, units):
        return min(int(units), self.max_vel) * self.counts_s_per_vel_unit

    def _acc(self, units):
        return min(int(units), self.max_acc) * self.counts_s2_per_acc_unit

    def _on_command(self, msg):
        # cb_joint2 / cb_joint3 (joint_control.py:222-248)
        if self.profile is None:
            return
        _angle, target = self.conv.command(msg.data)
        # set_position (epos_controller.py:220-237): el drive acepta las escrituras aunque esté en
        # falla, pero no ejecuta nada (CiA 402). Se aplican tras la latencia de las 5 escrituras SDO.
        self.pending.append((self.clock() + self.latency, float(target), self.sync_vel,
                             int(self.c['profile_acceleration']), int(self.c['profile_deceleration'])))

    def _on_sync(self, msg):
        self.sync_vel = max(int(self.c['sync_velocity_min']), int(msg.data))   # joint_control.py:213-217

    def _apply_setpoint(self, target, vel, acc, dec):
        self.profile.set_limits(vmax=self._vel(vel), accel=self._acc(acc), decel=self._acc(dec))
        self.profile.set_target(target)

    # ------------------------------------------------------------------------------------------
    def update(self, now, dt, measured_phys):
        if self.profile is None:
            return
        self.measured_counts = self._counts_from_physical(measured_phys)
        while self.pending and self.pending[0][0] <= now:
            _, target, vel, acc, dec = self.pending.popleft()
            if self.fault is not None:
                continue
            if self.mode == 'immediate' or self.profile.done:
                self._apply_setpoint(target, vel, acc, dec)
            elif self.buffered is None:
                # Bit 5 = 0 (epos_controller.py:235): espera a terminar el movimiento en curso.
                self.buffered = (target, vel, acc, dec)
            # Con el búfer lleno, el set-point nuevo no se acepta (CiA 402).
        if self.fault is not None:
            return
        self.profile.step(dt)
        if self.profile.done and self.buffered is not None:
            self._apply_setpoint(*self.buffered)
            self.buffered = None
        self._check_following_error(now, measured_phys)

    def _check_following_error(self, now, measured_phys):
        command = self._physical_from_counts(self.profile.x)
        self.history.append((now, command))
        while self.history and now - self.history[0][0] > self.tracking_window:
            self.history.popleft()
        # Gazebo sigue la orden con unos ms de retraso que el EPOS real no tiene: se compara contra
        # la orden más cercana dentro de la ventana sim_tracking_window_s.
        err_deg = min(abs(measured_phys - h) for _, h in self.history)
        err_counts = err_deg * self.conv.counts_per_deg() / abs(self.physical.scale)
        self.last_error_counts = err_counts
        if err_counts > self.window_counts:
            self._enter_fault(measured_phys)

    def _enter_fault(self, measured_phys):
        code = int(self.c['fault_code'])
        self.fault = {'code': code, 'time': self.clock()}
        self.fault_history.insert(0, code)
        del self.fault_history[5:]
        self.hold_phys = measured_phys   # fault_behavior: hold (DEFAULT)
        self.pending.clear()
        self.buffered = None

    def gazebo_command(self, measured_phys):
        if self.profile is None:
            return measured_phys
        if self.fault is not None:
            return self.hold_phys
        return self._physical_from_counts(self.profile.x)

    def publish_feedback(self):
        # publish_feedback (joint_control.py:286-305): counts y grados, también en falla.
        if self.measured_counts is None:
            return
        self.pub_raw.publish(Float64(data=self.measured_counts))
        self.pub_deg.publish(Float64(data=self.conv.counts_to_deg(self.measured_counts)))

    # ------------------------------------------------------------------------------------------
    @property
    def faulted(self):
        return self.fault is not None

    def fault_info(self):
        """Mismo formato que get_fault_info (epos_controller.py:287-346)."""
        code = self.fault['code'] if self.fault else 0
        bits = [int(b) for b in self.c['fault_error_register_bits']]
        return {
            'error_code': code,
            'error_name': EPOS_ERROR_NAMES.get(code, f'Unknown (0x{code:04X})'),
            'error_register_flags': [EPOS_ERROR_REGISTER_FLAGS[b] for b in sorted(EPOS_ERROR_REGISTER_FLAGS)
                                     if b in bits],
            'error_history': [{'code': c, 'name': EPOS_ERROR_NAMES.get(c, f'Unknown (0x{c:04X})')}
                              for c in self.fault_history],
        }

    def reset_fault(self, measured_phys):
        """Botón "Reiniciar driver": equivale a clear_errors() (epos_controller.py:137-143).

        Limpia la falla y vuelve a habilitar el drive SIN redeclarar la posición (no descalibra).
        """
        if self.fault is None:
            return False
        self.fault = None
        self.hold_phys = None
        self.history.clear()
        self.profile.reset(self._counts_from_physical(measured_phys))
        return True

    def status(self):
        s = {'driver': 'epos_ppm', 'faulted': self.faulted,
             'following_error_counts': round(self.last_error_counts, 1)}
        if self.fault:
            info = self.fault_info()
            s['fault'] = f'0x{info["error_code"]:04X} {info["error_name"]}'
        return s
