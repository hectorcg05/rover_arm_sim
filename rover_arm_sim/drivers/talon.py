"""Modelo `talon_motion_magic`: emula J1/J4 de drive_can_tank (Talon SRX en Motion Magic)."""

from std_msgs.msg import Float64

from ..conversions import TalonConversion
from ..profiles import TrapezoidProfile
from .base import DriverModel

TALON_VELOCITY_UNIT_S = 0.1   # Motion Magic mide velocidad en ticks por 100 ms (unidad de Phoenix 5)


class TalonMotionMagic(DriverModel):
    def __init__(self, name, cfg, node, clock):
        super().__init__(name, cfg, node, clock)
        c = cfg['talon_motion_magic']
        self.c = c
        self.conv = TalonConversion.from_cfg(c)
        self.cruise = float(c['cruise_velocity'])
        self.accel = float(c['acceleration'])
        self.profile = None
        self.was_enabled = False
        self.pending_target = None
        self.stall_since = None
        self.stalled = False
        self.measured_ticks = None
        self.pub_deg = node.create_publisher(Float64, cfg['feedback_deg_topic'], 10)
        self.pub_raw = node.create_publisher(Float64, cfg['feedback_raw_topic'], 10)
        node.create_subscription(Float64, cfg['command_topic'], self._on_command, 10)
        node.create_subscription(Float64, cfg['sync_velocity_topic'], self._on_sync, 10)

    @staticmethod
    def driver_limits_deg(cfg):
        c = cfg['talon_motion_magic']
        return float(c['deg_min']), float(c['deg_max'])

    # ------------------------------------------------------------------------------------------
    def _ticks_from_physical(self, phys_deg):
        # GetSelectedSensorPosition() entrega un entero (drive_can_tank.cpp:414).
        return float(round(self.conv.deg_to_ticks(self.physical.to_driver(phys_deg))))

    def _vmax(self):
        return self.cruise / TALON_VELOCITY_UNIT_S

    def _amax(self):
        return self.accel / TALON_VELOCITY_UNIT_S

    def start(self, measured_phys):
        self.measured_ticks = self._ticks_from_physical(measured_phys)
        # El perfil arranca en la posición exacta (el redondeo a enteros es solo del feedback).
        exact = self.conv.deg_to_ticks(self.physical.to_driver(measured_phys))
        self.profile = TrapezoidProfile(exact, self._vmax(), self._amax())
        # Mismo texto que drive_can_tank.cpp:262-269
        c = self.c
        self.node.get_logger().info(
            f'{self.log_name} limits: deg=[{c["deg_min"]:.1f}, {c["deg_max"]:.1f}] '
            f'ticks=[{c["ticks_min"]:.1f}, {c["ticks_max"]:.1f}]')
        super().start(measured_phys)

    def _on_command(self, msg):
        ok, angle, target = self.conv.command(msg.data)
        if not ok:
            # drive_can_tank.cpp:371-372 (mismo texto). El comando se ignora y no renueva FeedEnable.
            self.node.get_logger().warn(
                f'{self.log_name} angle {angle:.1f} out of range '
                f'[{self.conv.deg_min:.1f}, {self.conv.deg_max:.1f}]')
            return
        self.group.feed_enable(self.group.enable_timeout)
        if self.profile is not None:
            self.profile.set_target(target)
            self.pending_target = None

    def _on_sync(self, msg):
        # j1_vel_callback / j4_vel_callback (drive_can_tank.cpp:333-343): recorta y trunca a entero.
        vel = max(float(self.c['sync_velocity_min']), min(msg.data, float(self.c['sync_velocity_max'])))
        self.cruise = float(int(vel))
        if self.profile is not None:
            self.profile.set_limits(vmax=self._vmax())

    # ------------------------------------------------------------------------------------------
    def update(self, now, dt, measured_phys):
        if self.profile is None:
            return
        self.measured_ticks = self._ticks_from_physical(measured_phys)
        enabled = self.group.enabled(now)
        if not enabled:
            if self.was_enabled:
                # Neutral al vencer FeedEnable: se queda donde está (neutral_behavior: hold, DEFAULT).
                # Phoenix conserva la última orden y la retoma al habilitarse otra vez.
                self.pending_target = self.profile.target
                self.profile.reset(self.measured_ticks)
            self.was_enabled = False
            return
        if not self.was_enabled and self.pending_target is not None:
            self.profile.set_target(self.pending_target)
            self.pending_target = None
        self.was_enabled = True
        self.profile.step(dt)
        # Aviso de "atascado" (solo para la ventana; el Talon real sigue empujando sin avisar, C21).
        lag_deg = abs(self.profile.x - self.measured_ticks) / self.conv.ticks_per_deg()
        if lag_deg > float(self.c['stall_warning_deg']):
            self.stall_since = self.stall_since if self.stall_since is not None else now
            self.stalled = now - self.stall_since >= float(self.c['stall_warning_time_s'])
        else:
            self.stall_since = None
            self.stalled = False

    def gazebo_command(self, measured_phys):
        if self.profile is None:
            return measured_phys
        return self.physical.to_physical(self.conv.ticks_to_deg(self.profile.x))

    def publish_feedback(self):
        if self.measured_ticks is None:
            return
        # publish_feedback (drive_can_tank.cpp:413-430): ticks crudos y grados.
        self.pub_raw.publish(Float64(data=self.measured_ticks))
        self.pub_deg.publish(Float64(data=self.conv.ticks_to_deg(self.measured_ticks)))

    def status(self):
        return {'driver': 'talon_motion_magic', 'stalled': self.stalled,
                'enabled': self.was_enabled}
