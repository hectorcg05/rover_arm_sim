"""Modelo `torque`: contrato de epos4_drive (EPOS4 en CST), para cuando el equipo migre (§E.13).

Hoy ningún joint lo usa. Copia el comportamiento de epos4_drive/src/epos4_torque_node.cpp:
  * comando de par de motor en N·m, recortado a ±max_torque_nm (:184-192);
  * watchdog: si no llegan comandos en watchdog_timeout_ms, par 0 (:230-237);
  * si el motor pasa de max_speed_rpm, par 0 con WARN (:240-246);
  * feedback a loop_hz: ticks = initial_ticks + sign·raw, grados = initial + sign·raw/cpr_out·360
    (:207-226), con cpr_out = 4·encoder_cpr/gear_ratio.
En Gazebo el joint pasa a control por esfuerzo: par en el joint = sign · par de motor · joint_gear_ratio.
No tiene límites de posición propios (el real tampoco): los límites del joint salen de `limits_deg`.
"""

import math

from std_msgs.msg import Float64

from .base import DriverModel


class Torque(DriverModel):
    command_interface = 'effort'

    def __init__(self, name, cfg, node, clock):
        super().__init__(name, cfg, node, clock)
        c = cfg['torque']
        self.c = c
        self.max_torque = float(c['max_torque_nm'])
        self.watchdog = float(c['watchdog_timeout_ms']) / 1000.0
        self.max_rpm = float(c['max_speed_rpm'])
        self.sign = float(c['sign'])
        self.joint_gear = float(c['joint_gear_ratio'])
        self.cpr_out = 4.0 * float(c['encoder_cpr']) / float(c['gear_ratio'])
        self.initial = float(c['initial_angle'])
        self.cmd = 0.0
        self.last_cmd_time = -1e9
        self.torque_out = 0.0
        self.start_phys = None
        self.measured_phys = None
        self.last_phys = None
        self.motor_rpm = 0.0
        self._last_warn = -1e9
        node.create_subscription(Float64, c['command_topic'], self._on_command, 10)
        self.pub_deg = node.create_publisher(Float64, c['feedback_deg_topic'], 10)
        self.pub_raw = node.create_publisher(Float64, c['feedback_raw_topic'], 10)

    @staticmethod
    def driver_limits_deg(cfg):
        lo, hi = cfg['torque']['limits_deg']
        return float(lo), float(hi)

    def start(self, measured_phys):
        self.start_phys = measured_phys
        self.measured_phys = measured_phys
        self.last_phys = measured_phys
        self.node.create_timer(1.0 / float(self.c['loop_hz']), self.publish_feedback)
        super().start(measured_phys)

    def _on_command(self, msg):
        self.cmd = max(-self.max_torque, min(self.max_torque, msg.data))
        self.last_cmd_time = self.clock()

    def _raw_ticks(self):
        # El encoder cuenta desde 0 al encender: vueltas de motor desde la postura de arranque.
        motor_deg = (self.measured_phys - self.start_phys) * self.joint_gear * self.sign
        return int(round(motor_deg / 360.0 * 4.0 * float(self.c['encoder_cpr'])))

    def update(self, now, dt, measured_phys):
        if self.start_phys is None:
            return
        self.measured_phys = measured_phys
        if dt > 0:
            joint_deg_s = (measured_phys - self.last_phys) / dt
            self.motor_rpm = joint_deg_s / 360.0 * 60.0 * self.joint_gear
        self.last_phys = measured_phys
        torque = self.cmd
        if now - self.last_cmd_time > self.watchdog:
            torque = 0.0
        if abs(self.motor_rpm) > self.max_rpm:
            if now - self._last_warn > 0.5:
                self.node.get_logger().warn(
                    f'Overspeed detected ({int(self.motor_rpm)} RPM > {int(self.max_rpm)} RPM limit)! '
                    'Cutting torque to 0.')
                self._last_warn = now
            torque = 0.0
        self.torque_out = self.sign * torque

    def gazebo_command(self, measured_phys):
        return self.torque_out * self.joint_gear

    def publish_feedback(self):
        if self.measured_phys is None:
            return
        raw = self._raw_ticks()
        initial_ticks = self.initial / 360.0 * self.cpr_out
        self.pub_raw.publish(Float64(data=initial_ticks + self.sign * raw))
        self.pub_deg.publish(Float64(data=self.initial + self.sign * raw / self.cpr_out * 360.0))

    def status(self):
        return {'driver': 'torque', 'torque_nm': round(self.torque_out, 3),
                'motor_rpm': round(self.motor_rpm, 1) if math.isfinite(self.motor_rpm) else None}
