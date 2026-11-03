"""Modelo de Gazebo generado: mapeo URDF <-> software (§4.1), límites = drivers (K6), postura P2,
sin cámara vieja, colisiones simplificadas, gripper de dos mordazas, mundo de práctica y holguras
de la postura de arranque (criterio 8, parte estática)."""

import math
import os
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
from check_clearance import clearance  # noqa: E402

from rover_arm_sim import model_builder as mb  # noqa: E402

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'arm_sim.yaml')


@pytest.fixture(scope='module')
def cfg():
    with open(CONFIG) as f:
        return yaml.safe_load(f)


@pytest.fixture(scope='module')
def generated(cfg, tmp_path_factory):
    out = tmp_path_factory.mktemp('model')
    paths = mb.generate(cfg, str(out))
    return paths, ET.parse(paths['urdf']).getroot(), ET.parse(paths['world']).getroot()


def joint(robot, name):
    return next(j for j in robot.findall('joint') if j.get('name') == name)


def test_mapping_matches_report_4_1(cfg):
    """θ1 = −q1; θ2 = 90 − q2; θ3 = −2.133 − q3; θ4 = 2.133 − q4 (grados)."""
    j = cfg['joints']
    for q in (-40.0, 0.0, 33.0, 170.0):
        assert math.degrees(mb.physical_to_urdf_rad(j['joint1'], q)) == pytest.approx(-q)
        assert math.degrees(mb.physical_to_urdf_rad(j['joint2'], q)) == pytest.approx(90 - q)
        assert math.degrees(mb.physical_to_urdf_rad(j['joint3'], q)) == pytest.approx(-2.133 - q)
        assert math.degrees(mb.physical_to_urdf_rad(j['joint4'], q)) == pytest.approx(2.133 - q)
        assert mb.urdf_rad_to_physical(j['joint3'], mb.physical_to_urdf_rad(j['joint3'], q)) == pytest.approx(q)


def test_joint_limits_are_driver_limits(cfg, generated):
    _, robot, _ = generated
    expected = {'Joint_1': (-90.0, 85.0),                    # J1 [−85, 90]
                'Joint_2': (90 - 170 - 1000 / 5760.0, 90 + 5 + 1000 / 5760.0),   # J2 [−5, 170] ± 1000 counts
                'Joint_4': (2.133 - 150, 2.133 + 150),        # J4 ±150
                'Joint_5': (-90.0, 90.0)}
    for name, (lo, hi) in expected.items():
        lim = joint(robot, name).find('limit')
        assert math.degrees(float(lim.get('lower'))) == pytest.approx(lo, abs=1e-3), name
        assert math.degrees(float(lim.get('upper'))) == pytest.approx(hi, abs=1e-3), name
    # J2 permite la postura real de arranque (170°), que el CAD no permite (brazo.urdf:67)
    lim = joint(robot, 'Joint_2').find('limit')
    assert float(lim.get('lower')) <= mb.physical_to_urdf_rad(cfg['joints']['joint2'], 170.0)


def test_initial_pose_is_p2(cfg, generated):
    _, robot, _ = generated
    rc = robot.find('ros2_control')
    initial = {j.get('name'): float(j.find("state_interface[@name='position']/param").text)
               for j in rc.findall('joint')}
    for key, q in cfg['startup']['pose_deg'].items():
        name = cfg['joints'][key]['gazebo']['urdf_joint']
        assert initial[name] == pytest.approx(mb.physical_to_urdf_rad(cfg['joints'][key], q), abs=1e-6)
    assert math.degrees(initial['Joint_2']) == pytest.approx(-80.0, abs=1e-4)


def test_no_old_camera_and_rover_fixed(generated):
    paths, robot, _ = generated
    names = {l.get('name') for l in robot.findall('link')}
    assert not any('camera' in n for n in names)
    assert 'sensor' not in open(paths['urdf']).read()
    assert joint(robot, 'world_to_chasis').get('type') == 'fixed'
    assert all(j.get('type') == 'fixed' for j in robot.findall('joint') if j.get('name').endswith('_wheel_joint'))


def test_collisions_are_primitives_and_self_collide(generated):
    paths, robot, _ = generated
    for link in robot.findall('link'):
        for col in link.findall('collision'):
            assert col.find('geometry/mesh') is None, link.get('name')
    assert '<self_collide>true</self_collide>' in open(paths['urdf']).read()


def test_gripper_has_two_prismatic_jaws(cfg, generated):
    _, robot, _ = generated
    for key in ('finger_a', 'finger_b'):
        j = joint(robot, cfg['gripper']['fingers'][key]['joint'])
        assert j.get('type') == 'prismatic'
        lo, hi = float(j.find('limit').get('lower')), float(j.find('limit').get('upper'))
        assert 2 * lo + cfg['gripper']['fingers']['cad_opening_m'] == pytest.approx(0.0)
        assert 2 * hi + cfg['gripper']['fingers']['cad_opening_m'] == pytest.approx(cfg['gripper']['stroke_m'])


def test_controllers_follow_driver_models(cfg, generated):
    paths, _, _ = generated
    with open(paths['controllers']) as f:
        c = yaml.safe_load(f)
    assert c[mb.POSITION_CONTROLLER]['ros__parameters']['joints'] == mb.command_groups(cfg)['position']
    assert c['controller_manager']['ros__parameters']['update_rate'] == cfg['model']['controller_update_rate_hz']
    assert mb.EFFORT_CONTROLLER not in c                     # hoy ningún joint es de torque


def test_world_has_practice_objects(cfg, generated):
    _, _, world = generated
    models = {m.get('name'): m for m in world.find('world').findall('model')}
    objs = {o['name']: o for o in cfg['world']['objects']}
    assert len(objs) >= 5 and set(objs) <= set(models)
    assert objs['caja_5kg']['mass'] == pytest.approx(5.0)
    assert max(objs['caja_5kg']['size'][:2]) <= 0.17 or min(objs['caja_5kg']['size'][:2]) <= 0.17
    handle = [p for p in objs['contenedor_asa']['parts'] if p['shape'] == 'cylinder'][0]
    assert 2 * handle['radius'] == pytest.approx(0.075)
    assert objs['piedra']['shape'] == 'mesh'
    mass = float(models['caja_5kg'].find('link/inertial/mass').text)
    assert mass == pytest.approx(5.0)


def test_start_pose_clearances(cfg, generated):
    """La postura de arranque no toca chasis, piso ni a sí misma con las colisiones de la sim."""
    paths, robot, _ = generated
    q = cfg['startup']['pose_deg']
    d = clearance(robot, cfg, q, step=0.005)
    gaps = {k: v for k, v in d.items() if not k.endswith('dentro_chasis')}
    assert min(gaps.values()) > 0.005, sorted(gaps.items(), key=lambda kv: kv[1])[:3]
    assert all(v == 0.0 for k, v in d.items() if k.endswith('dentro_chasis'))
    assert d['Link_2-rover'] > 0.01


def test_switching_a_joint_to_torque_is_config_only(cfg, tmp_path):
    """Cambiar el modelo de driver de J2 a `torque` (epos4_drive) solo con el YAML: el modelo y los
    controladores se generan con control por esfuerzo para ese joint, sin tocar código."""
    import copy
    c = copy.deepcopy(cfg)
    c['joints']['joint2']['driver_model'] = 'torque'
    c['joints']['joint2']['torque'] = dict(cfg['torque_model_template'])
    groups = mb.command_groups(c)
    assert 'Joint_2' in groups['effort'] and 'Joint_2' not in groups['position']
    paths = mb.generate(c, str(tmp_path))
    robot = ET.parse(paths['urdf']).getroot()
    j2 = next(j for j in robot.find('ros2_control').findall('joint') if j.get('name') == 'Joint_2')
    assert j2.find('command_interface').get('name') == 'effort'
    with open(paths['controllers']) as f:
        ctl = yaml.safe_load(f)
    assert ctl[mb.EFFORT_CONTROLLER]['ros__parameters']['joints'] == ['Joint_2']
    assert mb.controller_names(c)[-1] == mb.EFFORT_CONTROLLER
    lim = joint(robot, 'Joint_2').find('limit')
    assert math.degrees(float(lim.get('upper'))) == pytest.approx(95.0, abs=1e-3)   # limits_deg [-5, 170]
