"""Simulación del brazo del rover: un solo comando para todo.

    ros2 launch rover_arm_sim arm_sim.launch.py                       # todo, con ventanas
    ros2 launch rover_arm_sim arm_sim.launch.py gazebo_gui:=false      # Gazebo sin ventana
    ros2 launch rover_arm_sim arm_sim.launch.py joystick_window:=false # con el Attack 3 físico

Orden de arranque (reporte §5.3-4):
  1. Genera el URDF, el mundo y los controladores a partir de config/arm_sim.yaml.
  2. Gazebo + robot_state_publisher + spawn + controladores + puentes del gripper.
  3. Emulador de drivers (reemplaza el hardware).
  4. Espera el primer feedback de J1..J4 y entonces lanza los launch REALES, sin cambios:
     arm_controller/launch/arm_launch.py y simple_drive2/launch/drive_arm_teleop_launch.py.
  5. Ventana "Attack 3 virtual" (opcional).
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, EmitEvent,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rover_arm_sim import model_builder as mb


def _bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('1', 'true', 'yes', 'si', 'sí')


def _setup(context):
    config = LaunchConfiguration('config').perform(context) or mb.default_config_path()
    cfg = mb.load_config(config)
    out_dir = os.path.expanduser(LaunchConfiguration('output_dir').perform(context))
    paths = mb.generate(cfg, out_dir)
    with open(paths['urdf']) as f:
        robot_description = f.read()
    gazebo_gui = _bool(context, 'gazebo_gui')
    actions = [LogInfo(msg=f'rover_arm_sim: configuración {config}; modelo generado en {out_dir}')]

    # Rutas de mallas (package://) y del plugin GraspWeld para Gazebo
    for pkg in ('mi_rover_description', 'rover_arm_sim'):
        share_parent = os.path.dirname(get_package_share_directory(pkg))
        actions.append(AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', share_parent))
        actions.append(AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', share_parent))
    plugin_dir = os.path.join(get_package_prefix('rover_arm_sim'), 'lib')
    actions.append(AppendEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', plugin_dir))
    actions.append(AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_dir))

    gz_args = f'-r {"" if gazebo_gui else "-s "}{paths["world"]}'
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()))

    actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                        output='log', parameters=[{'robot_description': robot_description}]))
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-name', cfg['model']['name'], '-file', paths['urdf']])
    actions.append(spawn)
    spawners = [Node(package='controller_manager', executable='spawner', output='screen',
                     arguments=[name, '--controller-manager', '/controller_manager',
                                '--controller-manager-timeout', '120'])
                for name in mb.controller_names(cfg)]
    actions.append(RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=spawners)))

    e = cfg['emulator']
    actions.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='rover_arm_sim_grasp_bridge',
        output='log',
        arguments=[f'{e["grasp_command_topic"]}@std_msgs/msg/String]ignition.msgs.StringMsg',
                   f'{e["grasp_state_topic"]}@std_msgs/msg/String[ignition.msgs.StringMsg']))

    # Sin `name=`: un __node:= renombraría TODOS los nodos del proceso (arm_drive_ctre, dual_epos_node...).
    actions.append(Node(package='rover_arm_sim', executable='driver_emulator', output='screen',
                        parameters=[{'config': config}]))

    wait = Node(package='rover_arm_sim', executable='wait_for_feedback', output='screen',
                parameters=[{'config': config}])
    actions.append(wait)

    real = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('arm_controller'), 'launch', 'arm_launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('simple_drive2'), 'launch', 'drive_arm_teleop_launch.py'))),
    ]
    if _bool(context, 'joystick_window'):
        joy_args = []
        mode = LaunchConfiguration('joystick_mode').perform(context)
        if mode:
            joy_args = ['--mode', mode]
        real.append(Node(package='rover_arm_sim', executable='virtual_joystick', output='screen',
                         arguments=['--config', config] + joy_args))

    def after_wait(event, _context):
        if event.returncode != 0:
            return [LogInfo(msg='rover_arm_sim: los drivers emulados no publicaron feedback; se detiene todo.'),
                    EmitEvent(event=Shutdown(reason='sin feedback'))]
        return [LogInfo(msg='rover_arm_sim: feedback listo; arrancando arm_manager y el teleop reales.')] + real

    actions.append(RegisterEventHandler(OnProcessExit(target_action=wait, on_exit=after_wait)))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gazebo_gui', default_value='true',
                              description='true: ventana de Gazebo; false: Gazebo sin ventana (pruebas)'),
        DeclareLaunchArgument('joystick_window', default_value='true',
                              description='true: ventana "Attack 3 virtual"; false: solo el Attack 3 físico'),
        DeclareLaunchArgument('joystick_mode', default_value='',
                              description='auto | uinput | topic (vacío = lo que diga el YAML)'),
        DeclareLaunchArgument('config', default_value='',
                              description='YAML de parámetros (vacío = config/arm_sim.yaml instalado)'),
        DeclareLaunchArgument('output_dir', default_value='~/.ros/rover_arm_sim',
                              description='carpeta donde se escriben el URDF y el mundo generados'),
        OpaqueFunction(function=_setup),
    ])
