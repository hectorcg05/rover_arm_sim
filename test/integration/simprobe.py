"""Herramientas para las pruebas de integración contra una simulación en marcha.

No es parte de la sim: solo publica entradas (como lo haría un operador) y registra lo que pasa.
La cinemática para calcular la punta es la clase ArmKinematics REAL, extraída de arm_manager.py.
"""

import ast
import json
import math
import os
import threading
import time

import rclpy
from rcl_interfaces.msg import Log
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Pose

SRC = os.path.expanduser('~/ros2_ws/src')


def real_kinematics():
    src = open(os.path.join(SRC, 'arm_controller/arm_controller/arm_manager.py')).read()
    cls = [n for n in ast.parse(src).body if isinstance(n, ast.ClassDef) and n.name == 'ArmKinematics'][0]
    ns = {'math': math}
    exec(compile(ast.Module([cls], []), 'arm_manager.py', 'exec'), ns)
    return ns['ArmKinematics'](0.10, 0.43, 0.43, 0.213)   # arm_launch.py:89-92


class Probe:
    def __init__(self, name='rover_arm_sim_probe'):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self.kin = real_kinematics()
        self.fb = {}
        self.cmd = {}
        self.pose = None
        self.status = {}
        self.logs = []
        self.mode = None
        self.lock = threading.Lock()
        for j in (1, 2, 3, 4):
            self.node.create_subscription(Float64, f'/arm_feedback/joint{j}_deg',
                                          lambda m, j=j: self.fb.__setitem__(j, m.data), 10)
        for j in (1, 2, 3, 4, 5):
            self.node.create_subscription(Float64, f'/arm_teleop/joint{j}',
                                          lambda m, j=j: self.cmd.__setitem__(j, m.data), 10)
        self.node.create_subscription(Pose, '/arm_cartesian/current_pose', self._pose, 10)
        self.node.create_subscription(String, '/rover_arm_sim/status', self._status, 10)
        self.node.create_subscription(String, '/arm_mode/current', lambda m: setattr(self, 'mode', m.data), 10)
        self.node.create_subscription(Log, '/rosout', self._log, 100)
        self.joy_pub = self.node.create_publisher(Joy, '/joy', 10)
        self.pubs = {}
        self._spin = True
        self._thread = threading.Thread(target=self._spinner, daemon=True)
        self._thread.start()

    def wait_ready(self, timeout=20.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if len(self.fb) == 4 and self.status:
                return True
            time.sleep(0.05)
        raise RuntimeError('sin feedback/estado de la simulación')

    def _spinner(self):
        while self._spin and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _pose(self, m):
        self.pose = (m.position.x, m.position.y, m.position.z, m.orientation.y)

    def _status(self, m):
        self.status = json.loads(m.data)

    def _log(self, m):
        with self.lock:
            self.logs.append((time.monotonic(), m.name, m.level, m.msg))

    def publish(self, topic, msg_type, value):
        if topic not in self.pubs:
            self.pubs[topic] = self.node.create_publisher(msg_type, topic, 10)
            # Espera a que un suscriptor de OTRO nodo descubra al publicador (esta sonda también
            # se suscribe a /arm_teleop/jointN y no debe contarse a sí misma).
            t0 = time.monotonic()
            while time.monotonic() - t0 < 5.0:
                others = [i for i in self.node.get_subscriptions_info_by_topic(topic)
                          if i.node_name != self.node.get_name()]
                if others and self.pubs[topic].get_subscription_count() > len(
                        [i for i in self.node.get_subscriptions_info_by_topic(topic)
                         if i.node_name == self.node.get_name()]):
                    break
                time.sleep(0.05)
            time.sleep(0.3)
        self.pubs[topic].publish(msg_type(data=value) if msg_type in (Float64, String) else value)

    def tip(self):
        """Punta (convención de arm_manager) calculada con el feedback MEDIDO."""
        f = self.kin.forward(self.fb[1], self.fb[2], self.fb[3], self.fb[4], 0.0)
        return f['x'], f['y'], f['z'], f['pitch']

    def joy(self, axes=(0.0, 0.0, 0.0), buttons=()):
        msg = Joy()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.axes = [float(a) for a in axes]
        b = [0] * 11
        for i in buttons:
            b[i] = 1
        msg.buttons = b
        self.joy_pub.publish(msg)

    def hold_joy(self, seconds, axes=(0.0, 0.0, 0.0), buttons=(), rate=20.0, record=None):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            self.joy(axes, buttons)
            if record is not None:
                record.append((time.monotonic(), self.tip(), self.pose))
            time.sleep(1.0 / rate)

    def wait(self, seconds, record=None, rate=20.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            if record is not None:
                record.append((time.monotonic(), self.tip(), self.pose))
            time.sleep(1.0 / rate)

    def logs_since(self, t0, names=None, min_level=0):
        with self.lock:
            return [l for l in self.logs if l[0] >= t0 and (names is None or l[1] in names) and l[2] >= min_level]

    def close(self):
        # Suelta todo antes de irse: el teleop real repite el último /joy para siempre (hallazgo 1).
        for _ in range(5):
            self.joy()
            time.sleep(0.05)
        self._spin = False
        self._thread.join(timeout=1)
        self.node.destroy_node()


def object_pose(name):
    """Posición (x, y, z) de un modelo de Gazebo, leída con `ign model`."""
    import re
    import subprocess
    out = subprocess.run(['ign', 'model', '-m', name, '-p'], capture_output=True,
                         text=True, timeout=15).stdout
    num = r'\s*(-?[\d.]+(?:e-?\d+)?)'
    m = re.search(r'\[' + num * 3 + r'\s*\]', out)
    return tuple(float(v) for v in m.groups()) if m else None


def move_joints(probe, targets, tol=0.3, timeout=15.0, j5_wait=0.0):
    """Manda ángulos directos a /arm_teleop/jointN (como en la prueba P3) y espera a que lleguen."""
    from std_msgs.msg import Float64
    for j, v in targets.items():
        probe.publish(f'/arm_teleop/joint{j}', Float64, float(v))
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if all(abs(probe.fb[j] - v) < tol for j, v in targets.items() if j in probe.fb):
            break
        if probe.status.get('faults'):
            break
        time.sleep(0.05)
    time.sleep(j5_wait)
    return {j: round(probe.fb.get(j, float('nan')), 2) for j in targets}, time.monotonic() - t0
