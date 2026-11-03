# rover_arm_sim

Simulación del brazo del rover (5 DOF + gripper) en Gazebo Fortress, para entrenar operadores de
teleoperación sin el hardware (v1).

```bash
cd ~/ros2_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select rover_arm_sim arm_controller simple_drive2 mi_rover_description
source install/setup.bash
ros2 launch rover_arm_sim arm_sim.launch.py            # gazebo_gui:=false / joystick_window:=false
```

- Operadores: `docs/sim_implementation/guia_operadores_sim.md`
- Equipo técnico (arquitectura, YAML, pruebas, resultados): `docs/sim_implementation/documentacion_sim.md`
- Todos los parámetros: `config/arm_sim.yaml` (cada valor con su fuente; `# PENDIENTE Px` / `# DEFAULT`).
- Pruebas: `colcon test --packages-select rover_arm_sim` (unitarias) y `test/integration/run_all.sh`
  (los 10 criterios, con la simulación).

No modifica ningún otro paquete: `arm_controller` y `simple_drive2` se lanzan con sus launch reales y
`mi_rover_description` solo se lee (mallas y CAD).
