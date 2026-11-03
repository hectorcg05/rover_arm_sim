"""Acceso de solo lectura al código REAL de los drivers, para comparar contra el emulador.

* drive_can_tank.cpp: se extraen my_map, ticks_to_deg_j1/j4 y los cuerpos de joint1_callback y
  joint4_callback, y se compilan con g++ en un arnés con stubs de Phoenix y rclcpp.
* joint_control.py: se importa con stubs de rclpy/canopen/epos_lib y se llaman sus métodos reales.
Nada de esto modifica los archivos originales.
"""

import ast
import os
import re
import subprocess
import sys
import tempfile
import types

SRC = os.environ.get('ROVER_SRC', os.path.expanduser('~/ros2_ws/src'))
DRIVE_CPP = os.path.join(SRC, 'bus_can_drive2/src/drive_can_tank.cpp')
DRIVE_YAML = os.path.join(SRC, 'bus_can_drive2/config/drive_can_tank_params.yaml')
JOINT_CONTROL = os.path.join(SRC, 'epos_control/epos_control/joint_control.py')
EPOS_LAUNCH = os.path.join(SRC, 'epos_control/launch/epos_control_launch.py')
EPOS_CONTROLLER = os.path.join(SRC, 'epos_control/epos_lib/epos_controller.py')


# ----------------------------------------------------------------------------------- C++ (Talon)
def _extract_function(src, signature_regex):
    m = re.search(signature_regex, src)
    if not m:
        raise LookupError(signature_regex)
    start = src.index('{', m.end() - 1)
    depth = 0
    for i in range(start, len(src)):
        depth += {'{': 1, '}': -1}.get(src[i], 0)
        if depth == 0:
            return src[m.start():i + 1], src[start + 1:i]
    raise LookupError(signature_regex)


HARNESS = r'''
#include <cstdio>
#include <cstdlib>
#include <string>
// ---- stubs de rclcpp / Phoenix (solo lo que usan los callbacks) ----
namespace std_msgs { namespace msg { struct Float64 { double data; using SharedPtr = Float64 *; }; } }
namespace ctre { namespace phoenix { namespace unmanaged { inline void FeedEnable(int) {} } } }
enum class ControlMode { MotionMagic };
struct Talon { double last = 0.0; bool set = false;
  void Set(ControlMode, double v) { last = v; set = true; } };
Talon srxArm1, srxArm5;
#define RCLCPP_WARN(logger, ...) do { std::printf("WARN "); std::printf(__VA_ARGS__); std::printf("\n"); } while (0)
// ---- código real copiado de drive_can_tank.cpp ----
%(globals)s
%(functions)s
struct Node { int get_logger() { return 0; }
  void joint1_callback(const std_msgs::msg::Float64::SharedPtr msg) { %(cb1)s }
  void joint4_callback(const std_msgs::msg::Float64::SharedPtr msg) { %(cb4)s }
};
int main(int argc, char ** argv) {
  j1_deg_min = std::atof(argv[1]); j1_deg_max = std::atof(argv[2]);
  j1_ticks_min = std::atof(argv[3]); j1_ticks_max = std::atof(argv[4]);
  j4_deg_min = std::atof(argv[5]); j4_deg_max = std::atof(argv[6]);
  j4_ticks_min = std::atof(argv[7]); j4_ticks_max = std::atof(argv[8]);
  Node n; char line[256];
  while (std::fgets(line, sizeof line, stdin)) {
    char kind[8]; double v;
    if (std::sscanf(line, "%%7s %%lf", kind, &v) != 2) continue;
    std::string k(kind);
    std_msgs::msg::Float64 m{v};
    if (k == "c1") { srxArm1.set = false; n.joint1_callback(&m);
      std::printf("RES %%d %%.17g\n", srxArm1.set ? 1 : 0, srxArm1.last); }
    else if (k == "c4") { srxArm5.set = false; n.joint4_callback(&m);
      std::printf("RES %%d %%.17g\n", srxArm5.set ? 1 : 0, srxArm5.last); }
    else if (k == "t1") std::printf("RES 1 %%.17g\n", ticks_to_deg_j1(v));
    else if (k == "t4") std::printf("RES 1 %%.17g\n", ticks_to_deg_j4(v));
    std::fflush(stdout);
  }
}
'''


class TalonHarness:
    """Compila las funciones reales de drive_can_tank.cpp y las ejecuta."""

    def __init__(self, limits):
        src = open(DRIVE_CPP).read()
        glob = '\n'.join(re.findall(r'^float j[14]_(?:deg|ticks)_(?:min|max)\s*=.*$', src, re.M))
        glob += '\n' + re.search(r'^double phys_error\s*=.*$', src, re.M).group(0)
        funcs = []
        for sig in (r'double my_map\([^)]*\)\s*\{', r'double ticks_to_deg_j1\([^)]*\)\s*\{',
                    r'double ticks_to_deg_j4\([^)]*\)\s*\{'):
            funcs.append(_extract_function(src, sig)[0])
        _, cb1 = _extract_function(src, r'void joint1_callback\([^)]*\)\s*\{')
        _, cb4 = _extract_function(src, r'void joint4_callback\([^)]*\)\s*\{')
        code = HARNESS % {'globals': glob, 'functions': '\n'.join(funcs), 'cb1': cb1, 'cb4': cb4}
        self.dir = tempfile.mkdtemp(prefix='talon_harness_')
        cpp = os.path.join(self.dir, 'harness.cpp')
        with open(cpp, 'w') as f:
            f.write(code)
        exe = os.path.join(self.dir, 'harness')
        subprocess.run(['g++', '-std=c++17', '-O0', cpp, '-o', exe], check=True, capture_output=True, text=True)
        args = [str(v) for v in limits]
        self.proc = subprocess.Popen([exe] + args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)

    def _ask(self, kind, value):
        self.proc.stdin.write(f'{kind} {value!r}\n')
        self.proc.stdin.flush()
        warn = None
        while True:
            line = self.proc.stdout.readline().strip()
            if line.startswith('WARN '):
                warn = line[5:]
                continue
            _, ok, v = line.split()
            return bool(int(ok)), float(v), warn

    def command(self, joint, angle):
        return self._ask('c1' if joint == 1 else 'c4', angle)

    def ticks_to_deg(self, joint, ticks):
        return self._ask('t1' if joint == 1 else 't4', ticks)[1]

    def close(self):
        self.proc.stdin.close()
        self.proc.wait(timeout=5)


# ----------------------------------------------------------------------------------- Python (EPOS)
def epos_launch_params():
    """Diccionario de parámetros de epos_control_launch.py (leído con ast, sin ejecutar launch)."""
    tree = ast.parse(open(EPOS_LAUNCH).read())
    for node in ast.walk(tree):
        if isinstance(node, ast.Dict) and any(isinstance(k, ast.Constant) and k.value == 'j2_node_id'
                                              for k in node.keys):
            return ast.literal_eval(node)
    raise LookupError('parámetros de dual_epos_node')


class _FakeEpos:
    def __init__(self):
        self.positions = []
        self.home = None

    def set_position(self, position, velocity=500, acceleration=1000, deceleration=1000):
        self.positions.append((position, velocity, acceleration, deceleration))

    def set_state(self, _state):
        pass

    def set_home_to_current(self, home_position=0):
        self.home = home_position
        return True

    def configure_ppm(self, **kwargs):
        self.ppm = kwargs


def load_joint_control():
    """Importa el módulo real joint_control.py con stubs para rclpy, canopen y epos_lib."""
    stubs = {}
    rclpy = types.ModuleType('rclpy')
    node_mod = types.ModuleType('rclpy.node')
    node_mod.Node = object
    rclpy.node = node_mod
    std_msgs = types.ModuleType('std_msgs')
    msg = types.ModuleType('std_msgs.msg')
    msg.Float64 = msg.Bool = object
    std_msgs.msg = msg
    epos_lib = types.ModuleType('epos_lib')
    ctrl = types.ModuleType('epos_lib.epos_controller')
    ctrl.EPOSController = lambda *a, **k: _FakeEpos()
    epos_lib.epos_controller = ctrl
    for name, mod in (('rclpy', rclpy), ('rclpy.node', node_mod), ('std_msgs', std_msgs),
                      ('std_msgs.msg', msg), ('canopen', types.ModuleType('canopen')),
                      ('epos_lib', epos_lib), ('epos_lib.epos_controller', ctrl)):
        stubs[name] = sys.modules.get(name)
        sys.modules[name] = mod
    try:
        ns = {'__name__': 'joint_control_real'}
        exec(compile(open(JOINT_CONTROL).read(), JOINT_CONTROL, 'exec'), ns)
    finally:
        for name, old in stubs.items():
            if old is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = old
    return ns


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(('info', m))

    def warn(self, m):
        self.lines.append(('warn', m))

    def error(self, m):
        self.lines.append(('error', m))


def real_epos_node(params):
    """Objeto DualEPOSNode real (sin __init__, que necesita el bus CAN) con su configuración."""
    ns = load_joint_control()
    cls = ns['DualEPOSNode']
    node = object.__new__(cls)
    for j in ('j2', 'j3'):
        setattr(node, f'{j}_cfg', {
            'node_id': params[f'{j}_node_id'], 'angle_min': params[f'{j}_angle_min'],
            'angle_max': params[f'{j}_angle_max'], 'counts_min': params[f'{j}_counts_min'],
            'counts_max': params[f'{j}_counts_max'], 'velocity': params[f'{j}_profile_velocity'],
            'accel': params[f'{j}_profile_acceleration'], 'decel': params[f'{j}_profile_deceleration'],
            'initial_angle': params[f'{j}_initial_angle']})
        setattr(node, f'{j}_sync_vel', params[f'{j}_profile_velocity'])
    node.is_homing = False
    node.epos_j2, node.epos_j3 = _FakeEpos(), _FakeEpos()
    logger = _Logger()
    node.get_logger = lambda: logger
    node.network = None
    node._logger_obj = logger
    return node, ns
