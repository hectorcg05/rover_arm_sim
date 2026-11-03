"""Comportamiento a nivel de "nodo driver" real (lo que comparten los joints de un mismo nodo).

* TalonGroup  = drive_can_tank (nodo arm_drive_ctre): habilitación global de 10 s (FeedEnable),
  logs de arranque y feedback de J1 y J4 en el mismo temporizador de 20 Hz.
* EposGroup   = dual_epos_node: logs de arranque, feedback a 20 Hz y monitor de fallas a 1 Hz.
* PlainGroup  = controladores que no están en el repo (servo de J5, gripper): sin logs propios.
"""

from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Twist

# Textos de error del EPOS. Copia de epos_control/epos_lib/epos_controller.py:4-69 (solo las
# entradas que la sim puede producir); test/test_sources.py verifica que coincidan con el original.
EPOS_ERROR_NAMES = {
    0x8611: 'Following error',
}
EPOS_ERROR_REGISTER_FLAGS = {
    0: 'Generic error',
    1: 'Current error',
    2: 'Voltage error',
    3: 'Temperature error',
    4: 'Communication error',
    5: 'Device profile-specific error',
    7: 'Motion error',
}


class DriverGroup:
    def __init__(self, name, cfg, node, clock):
        self.name = name
        self.cfg = cfg
        self.node = node
        self.clock = clock
        self.members = []

    def add(self, model):
        self.members.append(model)
        model.group = self

    def start(self):
        """Se llama cuando todos los miembros ya arrancaron (ya hay posición medida)."""

    def publish_feedback(self):
        for m in self.members:
            m.publish_feedback()


class PlainGroup(DriverGroup):
    pass


class TalonGroup(DriverGroup):
    def __init__(self, name, cfg, node, clock):
        super().__init__(name, cfg, node, clock)
        self.enabled_until = 0.0
        self.enable_timeout = float(cfg['enable_timeout_s'])
        self._joint_topics = set()
        # Mensajes en estos topics renuevan la habilitación (los de joints los renueva su modelo,
        # después de la revisión de rango, igual que drive_can_tank.cpp:370-375).
        self._extra_subs = []

    def register_joint_topic(self, topic):
        self._joint_topics.add(topic)

    def create_enable_subscriptions(self):
        for topic in self.cfg.get('enable_topics', []):
            if topic in self._joint_topics:
                continue
            self._extra_subs.append(self.node.create_subscription(
                Float64, topic, lambda _msg: self.feed_enable(self.enable_timeout), 10))
        self._extra_subs.append(self.node.create_subscription(
            Twist, self.cfg['cmd_vel_topic'],
            lambda _msg: self.feed_enable(float(self.cfg['cmd_vel_enable_timeout_s'])), 10))

    def feed_enable(self, seconds):
        """ctre::phoenix::unmanaged::FeedEnable(ms): habilita todos los Talon por `seconds`."""
        self.enabled_until = max(self.enabled_until, self.clock() + seconds)

    def enabled(self, now):
        return now < self.enabled_until

    def start(self):
        c = self.cfg
        # Mismo texto que drive_can_tank.cpp:327-329 (incluido el "J3" que el real imprime para J4).
        self.node.get_logger().info(
            f'CTRE node ready (J1=ID{c["can_id_arm1"]}, J3=ID{c["can_id_arm5"]}, '
            f'Barrena=ID{c["can_id_gripper"]}, Elevador=ID{c["can_id_lineal"]}, Drive)')
        self.node.create_timer(1.0 / float(c['feedback_rate_hz']), self.publish_feedback)


class EposGroup(DriverGroup):
    def __init__(self, name, cfg, node, clock):
        super().__init__(name, cfg, node, clock)
        self._faulted = {}
        self.node.create_subscription(Bool, '/do_homing', self._on_homing, 10)  # joint_control.py:124

    def announce_bus(self):
        # joint_control.py:96
        self.node.get_logger().info(f'Connected to CAN bus: {self.cfg["can_channel"]}')

    def start(self):
        self.node.create_timer(1.0 / float(self.cfg['feedback_rate_hz']), self.publish_feedback)
        self.node.create_timer(1.0 / float(self.cfg['fault_check_rate_hz']), self.check_faults)
        self.node.get_logger().info('Dual EPOS node initialized — feedback enabled.')  # joint_control.py:140

    def _on_homing(self, msg):
        if msg.data:
            self.node.get_logger().warn(
                '/do_homing no se emula en la simulación (en el rover corre el homing método 17).')

    def check_faults(self):
        """_check_faults (joint_control.py:311-330): avisa solo en las transiciones."""
        for m in self.members:
            was = self._faulted.get(m.name, False)
            now = m.faulted
            if now and not was:
                info = m.fault_info()
                self.node.get_logger().error(
                    f'{m.log_name} FAULT | '
                    f'codigo: 0x{info["error_code"]:04X} — {info["error_name"]} | '
                    f'registro: {info["error_register_flags"]} | '
                    f'historial: {[e["name"] for e in info["error_history"]]}')
            elif was and not now:
                self.node.get_logger().info(f'{m.log_name}: falla resuelta, drive operativo')
            self._faulted[m.name] = now


GROUP_KINDS = {
    'talon_group': TalonGroup,
    'epos_group': EposGroup,
    'plain': PlainGroup,
}
