"""Interfaz común de los modelos de driver (reporte §E.13).

Cada modelo:
  * escucha el mismo topic de comando que el driver real y aplica su lógica (validación,
    recortes, perfil, fallas) en las unidades nativas del driver;
  * entrega a Gazebo una orden (posición o esfuerzo) en grados físicos (o N·m);
  * publica el feedback con los mismos topics, unidades y frecuencia que el driver real.

El modelo NO sabe nada de Gazebo ni del URDF: el emulador convierte grados físicos a ángulos URDF.
"""


class PhysicalLayer:
    """Capa física (reporte §3.1): grados físicos = scale · grados del driver + offset_deg."""

    def __init__(self, cfg):
        cfg = cfg or {}
        self.scale = float(cfg.get('scale', 1.0))
        self.offset = float(cfg.get('offset_deg', 0.0))

    def to_physical(self, driver_deg):
        return self.scale * driver_deg + self.offset

    def to_driver(self, physical_deg):
        return (physical_deg - self.offset) / self.scale


class DriverModel:
    """Clase base. `name` es el nombre del joint en el YAML (joint1 ... joint5)."""

    #: 'position' (grados físicos) o 'effort' (N·m en el joint)
    command_interface = 'position'

    def __init__(self, name, cfg, node, clock):
        self.name = name
        self.cfg = cfg
        self.node = node            # nodo ROS del driver real que se emula (para logs y topics)
        self.clock = clock          # función que devuelve el tiempo actual en segundos
        self.physical = PhysicalLayer(cfg.get('physical'))
        self.log_name = cfg.get('log_name', name)
        self.started = False

    # ---- límites del driver, para los límites del joint en Gazebo (K6) ----------------------
    @staticmethod
    def driver_limits_deg(cfg):
        """(mín, máx) en grados del driver de lo que el driver deja hacer."""
        raise NotImplementedError

    # ---- ciclo de vida ----------------------------------------------------------------------
    def start(self, measured_phys):
        """Se llama una vez, con la primera posición medida en Gazebo (grados físicos)."""
        self.started = True

    def update(self, now, dt, measured_phys):
        """Avanza el modelo `dt` segundos. `measured_phys`: posición medida en Gazebo."""

    def gazebo_command(self, measured_phys):
        """Orden para Gazebo: grados físicos (posición) o N·m (esfuerzo)."""
        raise NotImplementedError

    # ---- fallas -----------------------------------------------------------------------------
    @property
    def faulted(self):
        return False

    def reset_fault(self, measured_phys):
        return False

    def status(self):
        return {}
