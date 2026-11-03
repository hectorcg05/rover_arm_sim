"""Cada valor VERIFICADO del YAML de la sim es copia del código real: esta prueba lo compara contra
el archivo original y falla si el equipo cambia el original (reporte §3.1, R11).

Excepción conocida (documentada): j2_initial_angle = 0.0 en el código contra 170° real (§E.12).
"""

import ast
import os
import re
import sys

import pytest
import yaml

sys.path.insert(0, os.path.dirname(__file__))
from real_code import (DRIVE_CPP, DRIVE_YAML, EPOS_CONTROLLER, JOINT_CONTROL, SRC,  # noqa: E402
                       epos_launch_params)

from rover_arm_sim.drivers.groups import EPOS_ERROR_NAMES, EPOS_ERROR_REGISTER_FLAGS  # noqa: E402

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'arm_sim.yaml')
KNOWN_EXCEPTIONS = {('joint2', 'initial_angle'): (0.0, 170.0)}


@pytest.fixture(scope='module')
def cfg():
    with open(CONFIG) as f:
        return yaml.safe_load(f)


def read(path):
    with open(os.path.join(SRC, path) if not os.path.isabs(path) else path) as f:
        return f.read()


def test_talon_yaml_copies(cfg):
    with open(DRIVE_YAML) as f:
        p = yaml.safe_load(f)['arm_drive_ctre']['ros__parameters']
    for j, prefix in (('joint1', 'j1'), ('joint4', 'j4')):
        c = cfg['joints'][j]['talon_motion_magic']
        for k in ('deg_min', 'deg_max', 'ticks_min', 'ticks_max'):
            assert c[k] == p[f'{prefix}_{k}'], (j, k)
    d = cfg['driver_nodes']['arm_drive_ctre']
    for k in ('can_id_arm1', 'can_id_arm5', 'can_id_gripper', 'can_id_lineal'):
        assert d[k] == p[k]


def test_talon_code_copies(cfg):
    src = read(DRIVE_CPP)
    j1 = cfg['joints']['joint1']['talon_motion_magic']
    j4 = cfg['joints']['joint4']['talon_motion_magic']
    assert f'srxArm1.ConfigMotionCruiseVelocity({j1["cruise_velocity"]}, 10)' in src
    assert f'srxArm1.ConfigMotionAcceleration({j1["acceleration"]}, 10)' in src
    assert f'srxArm5.ConfigMotionCruiseVelocity({j4["cruise_velocity"]}, 10)' in src
    assert f'srxArm5.ConfigMotionAcceleration({j4["acceleration"]}, 10)' in src
    assert f'std::max({j1["sync_velocity_min"]}, std::min(msg->data, {j1["sync_velocity_max"]}))' in src
    assert f'std::max({j4["sync_velocity_min"]}, std::min(msg->data, {j4["sync_velocity_max"]}))' in src
    assert re.search(r'double phys_error\s*=\s*0\s*;', src) and j1['phys_error'] == 0.0
    assert 'double angle = msg->data+phys_error;' in src and j1['add_phys_error']
    assert re.search(r'joint4_callback[^{]*\{\s*double angle = msg->data;', src) and not j4['add_phys_error']
    assert 'my_map(-angle, j4_deg_min' in src and j4['invert'] and not j1['invert']
    g = cfg['driver_nodes']['arm_drive_ctre']
    assert src.count('FeedEnable(10000)') >= 4 and g['enable_timeout_s'] == 10.0
    assert 'FeedEnable(5000)' in src and g['cmd_vel_enable_timeout_s'] == 5.0
    assert 'std::chrono::milliseconds(50)' in src and g['feedback_rate_hz'] == 20.0
    for j in ('joint1', 'joint4'):
        jc = cfg['joints'][j]
        for k in ('command_topic', 'sync_velocity_topic', 'feedback_deg_topic', 'feedback_raw_topic'):
            assert f'"{jc[k]}"' in src, (j, k)
    for t in g['enable_topics'] + [g['cmd_vel_topic']]:
        assert f'"{t}"' in src, t


def test_epos_launch_copies(cfg):
    p = epos_launch_params()
    for j, prefix in (('joint2', 'j2'), ('joint3', 'j3')):
        c = cfg['joints'][j]['epos_ppm']
        mapping = {'node_id': 'node_id', 'angle_min': 'angle_min', 'angle_max': 'angle_max',
                   'counts_min': 'counts_min', 'counts_max': 'counts_max',
                   'profile_velocity': 'profile_velocity', 'profile_acceleration': 'profile_acceleration',
                   'profile_deceleration': 'profile_deceleration', 'initial_angle': 'initial_angle'}
        for k, real in mapping.items():
            expected = p[f'{prefix}_{real}']
            if (j, k) in KNOWN_EXCEPTIONS:
                code_value, sim_value = KNOWN_EXCEPTIONS[(j, k)]
                assert expected == code_value, 'cambió el valor del código: revisar la excepción (§E.12)'
                assert c[k] == sim_value
                continue
            assert c[k] == expected, (j, k)
        assert cfg['joints'][j]['command_topic'] == p[f'{prefix}_topic']
    assert cfg['driver_nodes']['dual_epos_node']['can_channel'] == p['can_channel']


def test_joint_control_copies(cfg):
    src = read(JOINT_CONTROL)
    for j in ('joint2', 'joint3'):
        c = cfg['joints'][j]['epos_ppm']
        assert f"max_vel=cfg['velocity'] * {c['max_velocity_factor']}" in src
        assert f"max_accel=cfg['accel'] * {c['max_acceleration_factor']}" in src
        assert f"cfg['counts_max'])) - {c['drive_limit_margin_counts']}" in src
        assert f"cfg['counts_max'])) + {c['drive_limit_margin_counts']}" in src
        assert f"max({c['sync_velocity_min']}, int(msg.data))" in src
        jc = cfg['joints'][j]
        for k in ('sync_velocity_topic', 'feedback_deg_topic', 'feedback_raw_topic'):
            assert f"'{jc[k]}'" in src, (j, k)
        assert f"'{jc['log_name']}'" in src
    g = cfg['driver_nodes']['dual_epos_node']
    assert 'self.create_timer(0.05, self.publish_feedback)' in src and g['feedback_rate_hz'] == 20.0
    assert 'self.create_timer(1.0, self._check_faults)' in src and g['fault_check_rate_hz'] == 1.0


def test_epos_error_texts_match_epos_controller():
    tree = ast.parse(read(EPOS_CONTROLLER))
    found = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
            found[node.targets[0].id] = ast.literal_eval(node.value)
    for code, name in EPOS_ERROR_NAMES.items():
        assert found['_ERROR_CODES'][code] == name
    assert found['_ERROR_REGISTER_FLAGS'] == EPOS_ERROR_REGISTER_FLAGS


def test_following_error_window_default(cfg):
    for eds, j in (('epos_control/eds_files/flat90.eds', 'joint2'), ('epos_control/eds_files/joint3.eds', 'joint3')):
        m = re.search(r'\[6065\]\s*\n(?:[^\[]*\n)*?DefaultValue=(\d+)', read(eds))
        assert int(m.group(1)) == cfg['joints'][j]['epos_ppm']['following_error_window_counts']


def test_teleop_and_joy_copies(cfg):
    src = read('simple_drive2/launch/drive_arm_teleop_launch.py')
    j = cfg['joystick']
    assert f"'deadzone': {j['joy_deadzone']}" in src
    assert f"'autorepeat_rate': {j['joy_autorepeat_rate']}" in src
    assert f"'{cfg['gripper']['command_topic']}'" in read('simple_drive2/simple_drive2/drive_arm_teleop.py')
    am = read('arm_controller/arm_controller/arm_manager.py')
    assert f"'{cfg['joints']['joint5']['command_topic']}'" in am
    for k in ('joint1', 'joint2', 'joint3', 'joint4'):
        assert f"'{cfg['joints'][k]['feedback_deg_topic']}'" in am


def test_torque_template_matches_epos4_drive(cfg):
    with open(os.path.join(SRC, 'epos4_drive/config/epos4_params.yaml')) as f:
        p = yaml.safe_load(f)['epos4_torque_node']['ros__parameters']
    t = cfg['torque_model_template']
    for k in ('loop_hz', 'encoder_cpr', 'gear_ratio', 'max_speed_rpm', 'max_torque_nm', 'watchdog_timeout_ms'):
        assert float(t[k]) == float(p[k]), k


def test_startup_pose_is_p2(cfg):
    """P2 resuelto (GUIA_PENDIENTES.md): 0 / 170 / −146 / 0 / 0."""
    assert cfg['startup']['pose_deg'] == {'joint1': 0.0, 'joint2': 170.0, 'joint3': -146.0,
                                          'joint4': 0.0, 'joint5': 0.0}
    assert cfg['joints']['joint2']['epos_ppm']['initial_angle'] == 170.0
    assert cfg['joints']['joint3']['epos_ppm']['initial_angle'] == -146.0


def test_pending_values_are_marked(cfg):
    """Los valores de P1, P3 y P4 llevan su marca en el YAML (instrucción v1, regla 4)."""
    text = read(CONFIG)
    for key, mark in (('speed_deg_s: 16.6', 'PENDIENTE P3'), ('speed_deg_s: 30.0', 'PENDIENTE P3'),
                      ('setpoint_mode: immediate', 'PENDIENTE P3'), ('open_sign: 1.0', 'PENDIENTE P4'),
                      ('full_stroke_time_s: 4.0', 'PENDIENTE P4'), ('axis_codes:', 'PENDIENTE P1')):
        line = next(l for l in text.splitlines() if key in l)
        assert mark in line, line
    lines = text.splitlines()
    i = next(n for n, l in enumerate(lines) if l.strip().startswith('button_codes:'))
    assert 'PENDIENTE P1' in lines[i - 1]   # el comentario de los índices va justo arriba
