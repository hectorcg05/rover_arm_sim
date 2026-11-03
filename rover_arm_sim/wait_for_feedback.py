"""Espera el primer feedback de J1..J4 y termina. El launch arranca arm_manager hasta entonces
(si arm_manager no recibe feedback en 5 s, supone ceros: arm_manager.py:264-273)."""

import sys
import time

import rclpy
from std_msgs.msg import Float64

from . import model_builder as mb


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('rover_arm_sim_wait_for_feedback')
    node.declare_parameter('config', '')
    path = node.get_parameter('config').get_parameter_value().string_value or None
    cfg = mb.load_config(path)
    # Los 4 topics que lee arm_manager (arm_manager.py:193-196)
    topics = [cfg['joints'][k]['feedback_deg_topic'] for k in ('joint1', 'joint2', 'joint3', 'joint4')]
    seen = {}
    for t in topics:
        node.create_subscription(Float64, t, lambda msg, t=t: seen.setdefault(t, msg.data), 10)
    timeout = float(cfg['startup']['feedback_wait_timeout_s'])
    start = time.monotonic()
    node.get_logger().info(f'Esperando feedback de los drivers emulados en {topics} ...')
    while rclpy.ok() and len(seen) < len(topics):
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() - start > timeout:
            node.get_logger().error(f'Sin feedback en {timeout:.0f} s; faltan {sorted(set(topics) - set(seen))}')
            node.destroy_node()
            rclpy.shutdown()
            return 1
    values = ', '.join(f'{t.split("/")[-1]}={seen[t]:.1f}' for t in topics)
    node.get_logger().info(f'Feedback recibido ({values}); se puede lanzar arm_manager.')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
