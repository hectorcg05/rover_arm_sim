"""Modelo `servo`: J5 (roll). El controlador real no está en el repo (reporte N1, D5).

Escucha /arm_teleop/joint5 (Float64 en grados, arm_manager.py:216), recorta al rango y se mueve a
velocidad constante. No publica feedback (el real tampoco: arm_manager no lo lee).
"""

from std_msgs.msg import Float64

from .base import DriverModel


class Servo(DriverModel):
    def __init__(self, name, cfg, node, clock):
        super().__init__(name, cfg, node, clock)
        c = cfg['servo']
        self.c = c
        self.lo, self.hi = float(c['deg_min']), float(c['deg_max'])
        self.speed = float(c['speed_deg_s'])
        self.position = None
        self.target = None
        node.create_subscription(Float64, cfg['command_topic'], self._on_command, 10)

    @staticmethod
    def driver_limits_deg(cfg):
        c = cfg['servo']
        return float(c['deg_min']), float(c['deg_max'])

    def start(self, measured_phys):
        self.position = self.physical.to_driver(measured_phys)
        self.target = self.position
        super().start(measured_phys)

    def _on_command(self, msg):
        clamped = max(self.lo, min(self.hi, msg.data))
        if clamped != msg.data and self.c.get('warn_on_clamp', False):
            self.node.get_logger().warn(
                f'{self.log_name} {msg.data:.1f} fuera de [{self.lo:.1f}, {self.hi:.1f}], recortado')
        self.target = clamped

    def update(self, now, dt, measured_phys):
        if self.position is None:
            return
        step = self.speed * dt
        delta = self.target - self.position
        self.position += max(-step, min(step, delta))

    def gazebo_command(self, measured_phys):
        if self.position is None:
            return measured_phys
        return self.physical.to_physical(self.position)

    def publish_feedback(self):
        pass

    def status(self):
        return {'driver': 'servo'}
