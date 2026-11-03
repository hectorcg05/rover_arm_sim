"""Genera el URDF de simulación, el mundo de práctica y el YAML de controladores.

Todo sale de config/arm_sim.yaml y del CAD de mi_rover_description, que se LEE pero no se modifica:
  * eslabones, orígenes de joints, ejes, inercias y mallas visuales: brazo.urdf, chasis.urdf y el
    xacro del rover (montaje del brazo y posición de las llantas);
  * límites de cada joint: los del driver de ese joint (K6), no los del CAD;
  * colisiones: cajas simplificadas del YAML (tools/fit_collision.py);
  * gripper: cuerpo + dos mordazas prismáticas (mallas derivadas de Link_5.STL);
  * sin la cámara virtual del modelo viejo; el chasis queda fijo al mundo.

Uso por línea de comandos (para revisar lo que se genera):
    ros2 run rover_arm_sim build_model --out /tmp/rover_arm_sim
"""

import argparse
import copy
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from .drivers import model_class
from .drivers.base import PhysicalLayer

PACKAGE = 'rover_arm_sim'
GRASP_PLUGIN_FILE = 'rover_arm_sim_grasp_weld'
GRASP_PLUGIN_NAME = 'rover_arm_sim::GraspWeld'
POSITION_CONTROLLER = 'arm_position_controller'
EFFORT_CONTROLLER = 'arm_effort_controller'


# ---------------------------------------------------------------------------------------------
# Configuración y rutas
# ---------------------------------------------------------------------------------------------
def share_dir(package):
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(package)


def default_config_path():
    return os.path.join(share_dir(PACKAGE), 'config', 'arm_sim.yaml')


def load_config(path=None):
    with open(path or default_config_path()) as f:
        return yaml.safe_load(f)


# ---------------------------------------------------------------------------------------------
# Mapeo de ángulos (reporte §4.1)
# ---------------------------------------------------------------------------------------------
def physical_to_urdf_rad(jcfg, phys_deg):
    g = jcfg['gazebo']
    return math.radians(float(g['urdf_sign']) * phys_deg + float(g['urdf_offset_deg']))


def urdf_rad_to_physical(jcfg, theta_rad):
    g = jcfg['gazebo']
    return (math.degrees(theta_rad) - float(g['urdf_offset_deg'])) / float(g['urdf_sign'])


def joint_limits_rad(jcfg):
    """Límites del joint en Gazebo = lo que deja hacer su driver (K6), en radianes URDF."""
    lo, hi = model_class(jcfg['driver_model']).driver_limits_deg(jcfg)
    phys = PhysicalLayer(jcfg.get('physical'))
    margin = float(jcfg['gazebo'].get('limit_margin_deg', 0.0))
    p_lo, p_hi = sorted((phys.to_physical(lo), phys.to_physical(hi)))
    a = physical_to_urdf_rad(jcfg, p_lo - margin)
    b = physical_to_urdf_rad(jcfg, p_hi + margin)
    return min(a, b), max(a, b)


def gripper_finger_limits(cfg):
    g = cfg['gripper']
    cad = float(g['fingers']['cad_opening_m'])
    return (0.0 - cad) / 2.0, (float(g['stroke_m']) - cad) / 2.0


def finger_position_for_opening(cfg, opening):
    return (opening - float(cfg['gripper']['fingers']['cad_opening_m'])) / 2.0


def command_groups(cfg):
    """Joints de cada controlador, en el orden en que el emulador manda las órdenes."""
    position, effort = [], []
    for jcfg in cfg['joints'].values():
        name = jcfg['gazebo']['urdf_joint']
        if model_class(jcfg['driver_model']).command_interface == 'effort':
            effort.append(name)
        else:
            position.append(name)
    for key in ('finger_a', 'finger_b'):
        position.append(cfg['gripper']['fingers'][key]['joint'])
    return {'position': position, 'effort': effort}


# ---------------------------------------------------------------------------------------------
# Ayudas XML
# ---------------------------------------------------------------------------------------------
def _fmt(values):
    return ' '.join(f'{float(v):.6g}' for v in values)


def _origin(parent, xyz=(0, 0, 0), rpy=(0, 0, 0)):
    return ET.SubElement(parent, 'origin', xyz=_fmt(xyz), rpy=_fmt(rpy))


def _box_collision(link, name, box, rpy=(0, 0, 0)):
    col = ET.SubElement(link, 'collision', name=name)
    _origin(col, box['xyz'], box.get('rpy', rpy))
    ET.SubElement(ET.SubElement(col, 'geometry'), 'box', size=_fmt(box['size']))
    return col


def _box_inertial(link, mass, size, xyz=(0, 0, 0)):
    x, y, z = size
    inertial = ET.SubElement(link, 'inertial')
    _origin(inertial, xyz)
    ET.SubElement(inertial, 'mass', value=f'{mass:.6g}')
    ET.SubElement(inertial, 'inertia', ixx=f'{mass * (y * y + z * z) / 12:.6g}', ixy='0', ixz='0',
                  iyy=f'{mass * (x * x + z * z) / 12:.6g}', iyz='0',
                  izz=f'{mass * (x * x + y * y) / 12:.6g}')


def _gazebo_ref(robot, reference, **elements):
    gz = ET.SubElement(robot, 'gazebo', reference=reference)
    for tag, value in elements.items():
        ET.SubElement(gz, tag).text = str(value)
    return gz


def _read_cad(cfg):
    m = cfg['model']
    desc = share_dir(m['description_package'])
    arm = ET.parse(os.path.join(desc, m['arm_urdf'])).getroot()
    chassis = ET.parse(os.path.join(desc, m['chassis_urdf'])).getroot()
    import xacro
    rover = ET.fromstring(xacro.process_file(os.path.join(desc, m['rover_xacro'])).toxml())
    return arm, chassis, rover


# ---------------------------------------------------------------------------------------------
# URDF
# ---------------------------------------------------------------------------------------------
def build_urdf(cfg, controllers_path):
    arm, chassis_cad, rover = _read_cad(cfg)
    m = cfg['model']
    col = cfg['collision']
    robot = ET.Element('robot', name=m['name'])
    link_mu = float(m['link_mu'])

    # Mundo y chasis (fijo al mundo: el rover no se maneja en la sim)
    if m.get('fix_to_world', True):
        ET.SubElement(robot, 'link', name='world')
        j = ET.SubElement(robot, 'joint', name='world_to_chasis', type='fixed')
        ET.SubElement(j, 'parent', link='world')
        ET.SubElement(j, 'child', link='Chasis')
        _origin(j, m['rover_xyz'])
    chassis = ET.SubElement(robot, 'link', name='Chasis')
    cad_link = chassis_cad.find('link')
    chassis.append(copy.deepcopy(cad_link.find('inertial')))
    chassis.append(copy.deepcopy(cad_link.find('visual')))
    for box in col['chassis']:
        _box_collision(chassis, f'chasis_{box["name"]}', box)
    _gazebo_ref(robot, 'Chasis', mu1=link_mu, mu2=link_mu)

    # Llantas: visual del CAD, colisión cilíndrica, unidas fijas al chasis
    for wj in rover.findall('joint'):
        if not wj.get('name', '').endswith('_wheel_joint'):
            continue
        child = wj.find('child').get('link')
        cad_wheel = next(l for l in rover.findall('link') if l.get('name') == child)
        wl = ET.SubElement(robot, 'link', name=child)
        wl.append(copy.deepcopy(cad_wheel.find('inertial')))
        wl.append(copy.deepcopy(cad_wheel.find('visual')))
        c = ET.SubElement(wl, 'collision', name=f'{child}_collision')
        c.append(copy.deepcopy(cad_wheel.find('collision/origin')))
        ET.SubElement(ET.SubElement(c, 'geometry'), 'cylinder',
                      radius=f'{col["wheel"]["radius"]:.6g}', length=f'{col["wheel"]["length"]:.6g}')
        nj = ET.SubElement(robot, 'joint', name=wj.get('name'), type='fixed')
        ET.SubElement(nj, 'parent', link='Chasis')
        ET.SubElement(nj, 'child', link=child)
        nj.append(copy.deepcopy(wj.find('origin')))

    # Montaje del brazo (rover_completo.urdf.xacro:15-19)
    mount = next(j for j in rover.findall('joint') if j.find('child').get('link') == 'base')
    mj = ET.SubElement(robot, 'joint', name='chasis_a_brazo_joint', type='fixed')
    ET.SubElement(mj, 'parent', link='Chasis')
    ET.SubElement(mj, 'child', link='base')
    mj.append(copy.deepcopy(mount.find('origin')))

    # Eslabones del brazo
    by_urdf_joint = {j['gazebo']['urdf_joint']: (k, j) for k, j in cfg['joints'].items()}
    for cad in arm.findall('link'):
        name = cad.get('name')
        link = ET.SubElement(robot, 'link', name=name)
        link.append(copy.deepcopy(cad.find('inertial')))
        visual = copy.deepcopy(cad.find('visual'))
        if name == 'Link_5':
            visual.find('geometry/mesh').set('filename', f'package://{PACKAGE}/meshes/{m["gripper_body_mesh"]}')
        link.append(visual)
        for i, box in enumerate(col['links'].get(name, [])):
            _box_collision(link, f'{name}_collision_{i}', box)
        _gazebo_ref(robot, name, mu1=link_mu, mu2=link_mu)
    for cad in arm.findall('joint'):
        name = cad.get('name')
        _key, jcfg = by_urdf_joint[name]
        j = ET.SubElement(robot, 'joint', name=name, type=cad.get('type'))
        for tag in ('origin', 'parent', 'child', 'axis'):
            j.append(copy.deepcopy(cad.find(tag)))
        lo, hi = joint_limits_rad(jcfg)
        ET.SubElement(j, 'limit', lower=f'{lo:.6f}', upper=f'{hi:.6f}',
                      effort=f'{float(jcfg["gazebo"]["effort"]):.6g}',
                      velocity=f'{float(jcfg["gazebo"]["velocity"]):.6g}')

    # Mordazas del gripper (prismáticas sobre el riel de Link_5)
    g = cfg['gripper']
    fl = gripper_finger_limits(cfg)
    for key in ('finger_a', 'finger_b'):
        f = g['fingers'][key]
        box = col['fingers'][key]
        link = ET.SubElement(robot, 'link', name=f['link'])
        _box_inertial(link, float(g['fingers']['mass_kg']), box['size'], box['xyz'])
        v = ET.SubElement(link, 'visual')
        _origin(v)
        ET.SubElement(ET.SubElement(v, 'geometry'), 'mesh', filename=f'package://{PACKAGE}/meshes/{f["mesh"]}')
        mat = ET.SubElement(v, 'material', name='mordaza')
        ET.SubElement(mat, 'color', rgba='0.25 0.25 0.28 1')
        _box_collision(link, f'{f["link"]}_collision', box)
        j = ET.SubElement(robot, 'joint', name=f['joint'], type='prismatic')
        _origin(j)
        ET.SubElement(j, 'parent', link='Link_5')
        ET.SubElement(j, 'child', link=f['link'])
        ET.SubElement(j, 'axis', xyz=_fmt(f['axis']))
        ET.SubElement(j, 'limit', lower=f'{fl[0]:.6f}', upper=f'{fl[1]:.6f}',
                      effort=f'{float(g["fingers"]["effort_n"]):.6g}',
                      velocity=f'{float(g["fingers"]["velocity_m_s"]):.6g}')
        mu = float(g['fingers']['mu'])
        _gazebo_ref(robot, f['link'], mu1=mu, mu2=mu)

    # ros2_control: interfaz por joint según su modelo de driver; postura inicial (P2)
    rc = ET.SubElement(robot, 'ros2_control', name='rover_arm_sim', type='system')
    ET.SubElement(ET.SubElement(rc, 'hardware'), 'plugin').text = 'gz_ros2_control/GazeboSimSystem'
    pose = cfg['startup']['pose_deg']
    for key, jcfg in cfg['joints'].items():
        iface = model_class(jcfg['driver_model']).command_interface
        _rc_joint(rc, jcfg['gazebo']['urdf_joint'], iface, physical_to_urdf_rad(jcfg, float(pose[key])))
    p0 = finger_position_for_opening(cfg, float(cfg['startup']['gripper_opening_m']))
    for key in ('finger_a', 'finger_b'):
        _rc_joint(rc, g['fingers'][key]['joint'], 'position', p0)

    gz = ET.SubElement(robot, 'gazebo')
    if m.get('self_collide', True):
        ET.SubElement(gz, 'self_collide').text = 'true'
    plugin = ET.SubElement(gz, 'plugin', filename='gz_ros2_control-system',
                           name='gz_ros2_control::GazeboSimROS2ControlPlugin')
    ET.SubElement(plugin, 'parameters').text = controllers_path
    grasp = ET.SubElement(gz, 'plugin', filename=GRASP_PLUGIN_FILE, name=GRASP_PLUGIN_NAME)
    ET.SubElement(grasp, 'parent_link').text = 'Link_5'
    for key in ('finger_a', 'finger_b'):
        ET.SubElement(grasp, 'finger_link').text = g['fingers'][key]['link']
    ET.SubElement(grasp, 'command_topic').text = cfg['emulator']['grasp_command_topic']
    ET.SubElement(grasp, 'state_topic').text = cfg['emulator']['grasp_state_topic']
    ET.SubElement(grasp, 'min_contact_time').text = str(float(g['contact_min_time_s']))
    ET.indent(robot)
    return ET.tostring(robot, encoding='unicode')


def _rc_joint(rc, name, command_interface, initial):
    j = ET.SubElement(rc, 'joint', name=name)
    ET.SubElement(j, 'command_interface', name=command_interface)
    pos = ET.SubElement(j, 'state_interface', name='position')
    ET.SubElement(pos, 'param', name='initial_value').text = f'{initial:.6f}'
    ET.SubElement(j, 'state_interface', name='velocity')


# ---------------------------------------------------------------------------------------------
# Controladores (gz_ros2_control)
# ---------------------------------------------------------------------------------------------
def build_controllers_yaml(cfg):
    groups = command_groups(cfg)
    cm = {'update_rate': int(cfg['model']['controller_update_rate_hz']),
          'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
          POSITION_CONTROLLER: {'type': 'position_controllers/JointGroupPositionController'}}
    doc = {'controller_manager': {'ros__parameters': cm},
           'joint_state_broadcaster': {'ros__parameters': {
               'update_rate': int(cfg['model']['joint_state_rate_hz'])}},
           POSITION_CONTROLLER: {'ros__parameters': {'joints': groups['position']}}}
    if groups['effort']:
        cm[EFFORT_CONTROLLER] = {'type': 'effort_controllers/JointGroupEffortController'}
        doc[EFFORT_CONTROLLER] = {'ros__parameters': {'joints': groups['effort']}}
    return yaml.safe_dump(doc, sort_keys=False)


def controller_names(cfg):
    names = ['joint_state_broadcaster', POSITION_CONTROLLER]
    if command_groups(cfg)['effort']:
        names.append(EFFORT_CONTROLLER)
    return names


# ---------------------------------------------------------------------------------------------
# Mundo de práctica
# ---------------------------------------------------------------------------------------------
def _part_inertia(part, mass):
    shape = part['shape']
    if shape == 'box':
        x, y, z = part['size']
        return np.diag([mass * (y * y + z * z), mass * (x * x + z * z), mass * (x * x + y * y)]) / 12.0
    if shape == 'cylinder':
        r, h = float(part['radius']), float(part['length'])
        i = mass * (3 * r * r + h * h) / 12.0
        return np.diag([i, i, mass * r * r / 2.0])
    if shape == 'sphere':
        return np.eye(3) * 0.4 * mass * float(part['radius']) ** 2
    if shape == 'mesh':
        return _part_inertia({'shape': 'box', 'size': part['bbox']}, mass)
    raise ValueError(f'forma desconocida: {shape}')


def _part_volume(part):
    shape = part['shape']
    if shape == 'box':
        return float(np.prod(part['size']))
    if shape == 'cylinder':
        return math.pi * float(part['radius']) ** 2 * float(part['length'])
    if shape == 'sphere':
        return 4.0 / 3.0 * math.pi * float(part['radius']) ** 3
    if shape == 'mesh':
        return float(np.prod(part['bbox']))
    raise ValueError(f'forma desconocida: {shape}')


def _rpy_matrix(rpy):
    from .geometry import rpy_to_matrix
    return rpy_to_matrix(*rpy)


def object_parts(obj):
    if obj['shape'] == 'compound':
        return obj['parts']
    part = dict(obj)
    if obj['shape'] == 'mesh':
        part['xyz'] = [0.0, 0.0, float(obj['bbox'][2]) / 2.0]
    else:
        part['xyz'] = [0.0, 0.0, 0.0]
    return [part]


def object_mass_properties(obj):
    """Centro de masa e inercia de un objeto (masa repartida por volumen entre sus partes)."""
    parts = object_parts(obj)
    vols = np.array([_part_volume(p) for p in parts])
    masses = float(obj['mass']) * vols / vols.sum()
    centers = np.array([p.get('xyz', [0, 0, 0]) for p in parts], dtype=float)
    com = (masses[:, None] * centers).sum(0) / masses.sum()
    inertia = np.zeros((3, 3))
    for p, mi, c in zip(parts, masses, centers):
        r = _rpy_matrix(p.get('rpy', [0, 0, 0]))
        i_local = r @ _part_inertia(p, mi) @ r.T
        d = c - com
        inertia += i_local + mi * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return com, inertia


def _sdf_geometry(parent, part, mesh_dir):
    geo = ET.SubElement(parent, 'geometry')
    shape = part['shape']
    if shape == 'box':
        ET.SubElement(ET.SubElement(geo, 'box'), 'size').text = _fmt(part['size'])
    elif shape == 'cylinder':
        c = ET.SubElement(geo, 'cylinder')
        ET.SubElement(c, 'radius').text = str(part['radius'])
        ET.SubElement(c, 'length').text = str(part['length'])
    elif shape == 'sphere':
        ET.SubElement(ET.SubElement(geo, 'sphere'), 'radius').text = str(part['radius'])
    elif shape == 'mesh':
        ET.SubElement(ET.SubElement(geo, 'mesh'), 'uri').text = 'file://' + os.path.join(mesh_dir, part['mesh'])


def _sdf_object(world, obj, mesh_dir):
    model = ET.SubElement(world, 'model', name=obj['name'])
    yaw = math.radians(float(obj.get('yaw_deg', 0.0)))
    ET.SubElement(model, 'pose').text = _fmt(list(obj['xyz']) + [0.0, 0.0, yaw])
    link = ET.SubElement(model, 'link', name='link')
    com, inertia = object_mass_properties(obj)
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'pose').text = _fmt(list(com) + [0, 0, 0])
    ET.SubElement(inertial, 'mass').text = str(float(obj['mass']))
    i = ET.SubElement(inertial, 'inertia')
    for tag, (r, c) in (('ixx', (0, 0)), ('ixy', (0, 1)), ('ixz', (0, 2)),
                        ('iyy', (1, 1)), ('iyz', (1, 2)), ('izz', (2, 2))):
        ET.SubElement(i, tag).text = f'{inertia[r, c]:.6g}'
    color = _fmt(obj['color'])
    for k, part in enumerate(object_parts(obj)):
        pose = _fmt(list(part.get('xyz', [0, 0, 0])) + list(part.get('rpy', [0, 0, 0])))
        if part['shape'] == 'mesh':
            pose = _fmt([0, 0, 0, 0, 0, 0])
        c = ET.SubElement(link, 'collision', name=f'collision_{k}')
        ET.SubElement(c, 'pose').text = pose
        _sdf_geometry(c, part, mesh_dir)
        surface = ET.SubElement(ET.SubElement(c, 'surface'), 'friction')
        ode = ET.SubElement(surface, 'ode')
        ET.SubElement(ode, 'mu').text = str(float(obj['mu']))
        ET.SubElement(ode, 'mu2').text = str(float(obj['mu']))
        v = ET.SubElement(link, 'visual', name=f'visual_{k}')
        ET.SubElement(v, 'pose').text = pose
        _sdf_geometry(v, part, mesh_dir)
        mat = ET.SubElement(v, 'material')
        ET.SubElement(mat, 'ambient').text = color
        ET.SubElement(mat, 'diffuse').text = color


def build_world(cfg, mesh_dir):
    w = cfg['world']
    m = cfg['model']
    sdf = ET.Element('sdf', version='1.8')
    world = ET.SubElement(sdf, 'world', name=w['name'])
    physics = ET.SubElement(world, 'physics', name='sim', type='ignored')
    ET.SubElement(physics, 'max_step_size').text = str(float(m['physics_step_s']))
    ET.SubElement(physics, 'real_time_factor').text = str(float(m['real_time_factor']))
    dart = ET.SubElement(physics, 'dart')
    ET.SubElement(dart, 'collision_detector').text = m['collision_detector']
    for filename, name in (('ignition-gazebo-physics-system', 'ignition::gazebo::systems::Physics'),
                           ('ignition-gazebo-user-commands-system', 'ignition::gazebo::systems::UserCommands'),
                           ('ignition-gazebo-scene-broadcaster-system',
                            'ignition::gazebo::systems::SceneBroadcaster')):
        ET.SubElement(world, 'plugin', filename=filename, name=name)
    _gui(world, w)
    scene = ET.SubElement(world, 'scene')
    ET.SubElement(scene, 'ambient').text = '0.5 0.5 0.5 1'
    ET.SubElement(scene, 'background').text = '0.75 0.8 0.85 1'
    sun = ET.SubElement(world, 'light', type='directional', name='sol')
    ET.SubElement(sun, 'cast_shadows').text = 'true'
    ET.SubElement(sun, 'pose').text = '0 0 10 0 0 0'
    ET.SubElement(sun, 'diffuse').text = '0.85 0.85 0.85 1'
    ET.SubElement(sun, 'specular').text = '0.2 0.2 0.2 1'
    ET.SubElement(sun, 'direction').text = '-0.5 0.2 -0.9'
    ground = ET.SubElement(world, 'model', name='piso')
    ET.SubElement(ground, 'static').text = 'true'
    gl = ET.SubElement(ground, 'link', name='link')
    gc = ET.SubElement(gl, 'collision', name='collision')
    ET.SubElement(ET.SubElement(ET.SubElement(gc, 'geometry'), 'plane'), 'normal').text = '0 0 1'
    fr = ET.SubElement(ET.SubElement(ET.SubElement(gc, 'surface'), 'friction'), 'ode')
    ET.SubElement(fr, 'mu').text = str(float(w['ground_mu']))
    ET.SubElement(fr, 'mu2').text = str(float(w['ground_mu']))
    gv = ET.SubElement(gl, 'visual', name='visual')
    plane = ET.SubElement(ET.SubElement(gv, 'geometry'), 'plane')
    ET.SubElement(plane, 'normal').text = '0 0 1'
    ET.SubElement(plane, 'size').text = '20 20'
    gm = ET.SubElement(gv, 'material')
    ET.SubElement(gm, 'ambient').text = '0.62 0.56 0.48 1'
    ET.SubElement(gm, 'diffuse').text = '0.62 0.56 0.48 1'
    for obj in w['objects']:
        _sdf_object(world, obj, mesh_dir)
    ET.indent(sdf)
    return '<?xml version="1.0"?>\n' + ET.tostring(sdf, encoding='unicode')


def _gui(world, w):
    gui = ET.SubElement(world, 'gui', fullscreen='0')
    scene = ET.SubElement(gui, 'plugin', filename='GzScene3D', name='3D View')
    ig = ET.SubElement(scene, 'ignition-gui')
    ET.SubElement(ig, 'title').text = '3D View'
    ET.SubElement(ig, 'property', type='bool', key='showTitleBar').text = 'false'
    ET.SubElement(ig, 'property', type='string', key='state').text = 'docked'
    ET.SubElement(scene, 'engine').text = 'ogre2'
    ET.SubElement(scene, 'scene').text = 'scene'
    ET.SubElement(scene, 'ambient_light').text = '0.5 0.5 0.5'
    ET.SubElement(scene, 'background_color').text = '0.75 0.8 0.85'
    ET.SubElement(scene, 'camera_pose').text = _fmt(w['camera_pose'])
    for filename, title, anchor, extra in (
            ('WorldControl', 'World control', 'left',
             {'play_pause': 'true', 'step': 'true', 'start_paused': 'false'}),
            ('WorldStats', 'World stats', 'right',
             {'sim_time': 'true', 'real_time': 'true', 'real_time_factor': 'true', 'iterations': 'false'})):
        p = ET.SubElement(gui, 'plugin', filename=filename, name=title)
        ig = ET.SubElement(p, 'ignition-gui')
        ET.SubElement(ig, 'title').text = title
        ET.SubElement(ig, 'property', type='bool', key='showTitleBar').text = 'false'
        ET.SubElement(ig, 'property', type='bool', key='resizable').text = 'false'
        ET.SubElement(ig, 'property', type='double', key='height').text = '110' if anchor == 'right' else '72'
        ET.SubElement(ig, 'property', type='double', key='width').text = '290' if anchor == 'right' else '121'
        ET.SubElement(ig, 'property', type='double', key='z').text = '1'
        ET.SubElement(ig, 'property', type='string', key='state').text = 'floating'
        anchors = ET.SubElement(ig, 'anchors', target='3D View')
        ET.SubElement(anchors, 'line', own=anchor, target=anchor)
        ET.SubElement(anchors, 'line', own='bottom', target='bottom')
        for tag, value in extra.items():
            ET.SubElement(p, tag).text = value


# ---------------------------------------------------------------------------------------------
def generate(cfg, out_dir):
    """Escribe URDF, mundo y controladores en out_dir. Devuelve sus rutas."""
    os.makedirs(out_dir, exist_ok=True)
    controllers = os.path.join(out_dir, 'controllers.yaml')
    with open(controllers, 'w') as f:
        f.write(build_controllers_yaml(cfg))
    urdf = os.path.join(out_dir, 'rover_arm.urdf')
    with open(urdf, 'w') as f:
        f.write(build_urdf(cfg, controllers))
    world = os.path.join(out_dir, 'practica_brazo.sdf')
    with open(world, 'w') as f:
        f.write(build_world(cfg, os.path.join(share_dir(PACKAGE), 'meshes')))
    return {'urdf': urdf, 'world': world, 'controllers': controllers}


def main():
    ap = argparse.ArgumentParser(description='Genera el URDF y el mundo de la simulación del brazo')
    ap.add_argument('--config', default=None)
    ap.add_argument('--out', default=os.path.expanduser('~/.ros/rover_arm_sim'))
    args = ap.parse_args()
    paths = generate(load_config(args.config), args.out)
    for k, v in paths.items():
        print(f'{k}: {v}')


if __name__ == '__main__':
    main()
