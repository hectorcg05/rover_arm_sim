"""Emulador de drivers: reemplaza a drive_can_tank (J1, J4), dual_epos_node (J2, J3), al servo de J5
y al controlador del gripper. Es la única pieza que sustituye al hardware (reporte §1).

Arquitectura:
  * un nodo ROS por driver real (arm_drive_ctre, dual_epos_node, ...) con los mismos topics y los
    mismos mensajes de log;
  * un modelo de driver por joint, elegido en el YAML (driver_model), en unidades nativas;
  * un nodo puente (rover_arm_sim_emulator) que a `rate_hz` lee /joint_states de Gazebo, avanza los
    modelos y manda las órdenes a los controladores de gz_ros2_control.

Uso: ros2 run rover_arm_sim driver_emulator --ros-args -p config:=/ruta/arm_sim.yaml
"""

import json
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from . import model_builder as mb
from .drivers import model_class
from .drivers.gripper import DcGripper
from .drivers.groups import GROUP_KINDS, TalonGroup, EposGroup


class Emulator:
    def __init__(self, cfg):
        self.cfg = cfg
        self.clock = time.monotonic
        e = cfg['emulator']
        self.core = rclpy.create_node('rover_arm_sim_emulator')
        self.nodes = {name: rclpy.create_node(name) for name in cfg['driver_nodes']}
        self.groups = {}
        for name, gcfg in cfg['driver_nodes'].items():
            self.groups[name] = GROUP_KINDS[gcfg['kind']](name, gcfg, self.nodes[name], self.clock)
            if isinstance(self.groups[name], EposGroup):
                self.groups[name].announce_bus()
        self.models = {}
        for key, jcfg in cfg['joints'].items():
            cls = model_class(jcfg['driver_model'])
            group = self.groups[jcfg['driver_node']]
            model = cls(key, jcfg, self.nodes[jcfg['driver_node']], self.clock)
            group.add(model)
            if isinstance(group, TalonGroup):
                group.register_joint_topic(jcfg['command_topic'])
            self.models[key] = model
        for g in self.groups.values():
            if isinstance(g, TalonGroup):
                g.create_enable_subscriptions()
        gcfg = cfg['gripper']
        self.gripper = DcGripper(gcfg, self.nodes[gcfg['driver_node']], self.clock,
                                 cfg['startup']['gripper_opening_m'])
        self.finger_joints = [gcfg['fingers'][k]['joint'] for k in ('finger_a', 'finger_b')]
        groups = mb.command_groups(cfg)
        self.position_joints = groups['position']
        self.effort_joints = groups['effort']
        self.urdf_to_key = {j['gazebo']['urdf_joint']: k for k, j in cfg['joints'].items()}

        self.measured = {}          # joint urdf -> posición (rad o m)
        self.started = False
        self.last_time = None
        self.grasp_sent = None
        self.grasp_sent_time = 0.0

        self.pub_pos = self.core.create_publisher(Float64MultiArray, e['position_command_topic'], 10)
        self.pub_eff = (self.core.create_publisher(Float64MultiArray, e['effort_command_topic'], 10)
                        if self.effort_joints else None)
        self.pub_grasp = self.core.create_publisher(String, e['grasp_command_topic'], 10)
        self.pub_status = self.core.create_publisher(String, e['status_topic'], 10)
        self.core.create_subscription(JointState, e['joint_states_topic'], self._on_joint_states, 10)
        self.core.create_subscription(String, e['grasp_state_topic'], self._on_grasp_state, 10)
        self.core.create_service(Trigger, e['reset_service'], self._on_reset)
        self.core.create_timer(1.0 / float(e['rate_hz']), self._loop)
        self.core.create_timer(1.0 / float(e['status_rate_hz']), self._publish_status)
        self.core.get_logger().info(
            'Emulador de drivers listo; esperando /joint_states de Gazebo. Modelos: ' +
            ', '.join(f'{k}={j["driver_model"]}' for k, j in cfg['joints'].items()))

    # ------------------------------------------------------------------------------------------
    def all_nodes(self):
        return [self.core] + list(self.nodes.values())

    def _phys(self, key):
        jcfg = self.cfg['joints'][key]
        return mb.urdf_rad_to_physical(jcfg, self.measured[jcfg['gazebo']['urdf_joint']])

    def _on_joint_states(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.measured[name] = pos
        if not self.started:
            needed = list(self.urdf_to_key) + self.finger_joints
            if all(n in self.measured for n in needed):
                self._start()

    def _start(self):
        for key, model in self.models.items():
            model.start(self._phys(key))
        for g in self.groups.values():
            g.start()
        self.started = True
        self.last_time = self.clock()
        pose = ', '.join(f'{k}={self._phys(k):.1f}°' for k in self.models)
        self.core.get_logger().info(f'Gazebo conectado; postura medida: {pose}. Drivers emulados activos.')

    def _on_grasp_state(self, msg):
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        self.gripper.holding = state.get('attached', '')
        touching = state.get('touching', [[], []])
        self.gripper.touching = {'finger_a': touching[0] if touching else [],
                                 'finger_b': touching[1] if len(touching) > 1 else []}

    def _loop(self):
        if not self.started:
            return
        now = self.clock()
        dt = now - self.last_time
        self.last_time = now
        commands = {}
        for key, model in self.models.items():
            meas = self._phys(key)
            model.update(now, dt, meas)
            commands[self.cfg['joints'][key]['gazebo']['urdf_joint']] = (model, model.gazebo_command(meas), key)
        fingers = [self.measured[j] for j in self.finger_joints]
        self.gripper.update(now, dt, fingers)
        finger_cmd = dict(zip(self.finger_joints, self.gripper.finger_commands()))
        pos = []
        for name in self.position_joints:
            if name in finger_cmd:
                pos.append(finger_cmd[name])
            else:
                _model, value, key = commands[name]
                pos.append(mb.physical_to_urdf_rad(self.cfg['joints'][key], value))
        self.pub_pos.publish(Float64MultiArray(data=pos))
        if self.pub_eff is not None:
            # Par en el sentido físico del joint → sentido del eje URDF (mismo signo que el mapeo §4.1)
            self.pub_eff.publish(Float64MultiArray(data=[
                commands[n][1] * (1.0 if float(self.cfg['joints'][commands[n][2]]['gazebo']['urdf_sign']) > 0
                                  else -1.0)
                for n in self.effort_joints]))
        request = self.gripper.grasp_request()
        if request != self.grasp_sent or now - self.grasp_sent_time > 0.5:
            self.pub_grasp.publish(String(data=request))
            self.grasp_sent = request
            self.grasp_sent_time = now

    def _publish_status(self):
        joints = {k: m.status() for k, m in self.models.items()}
        faults = [m.log_name for m in self.models.values() if m.faulted]
        stalled = [m.log_name for m in self.models.values() if m.status().get('stalled')]
        fingers = [self.measured.get(j, 0.0) for j in self.finger_joints]
        status = {'started': self.started, 'faults': faults, 'stalled': stalled, 'joints': joints,
                  'gripper': self.gripper.status(fingers)}
        self.pub_status.publish(String(data=json.dumps(status)))

    def _on_reset(self, _request, response):
        cleared = []
        for key, model in self.models.items():
            if model.faulted and self.started and model.reset_fault(self._phys(key)):
                cleared.append(model.log_name)
        if cleared:
            response.message = (f'Falla limpiada en {", ".join(cleared)} sin redeclarar la posición '
                                '(equivale a clear_errors() del EPOS).')
        else:
            response.message = 'No había ningún driver en falla.'
        response.success = bool(cleared)
        self.core.get_logger().info(f'Reiniciar driver: {response.message}')
        return response


def main(args=None):
    rclpy.init(args=args)
    probe = rclpy.create_node('rover_arm_sim_emulator_config')
    probe.declare_parameter('config', '')
    path = probe.get_parameter('config').get_parameter_value().string_value or None
    probe.destroy_node()
    emulator = Emulator(mb.load_config(path))
    executor = SingleThreadedExecutor()
    for n in emulator.all_nodes():
        executor.add_node(n)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for n in emulator.all_nodes():
            n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
