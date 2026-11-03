#!/bin/bash
# Corre las 10 verificaciones de la v1. Cada grupo arranca una simulación NUEVA (Gazebo sin ventana,
# dominio ROS aislado) para que el estado de una prueba no afecte a la siguiente.
#
# Uso:  test/integration/run_all.sh [carpeta_de_resultados]
#       WITH_DISPLAY=1 test/integration/run_all.sh   # criterio 10 también con ventanas en pantalla
#       ONLY="c6 c9" test/integration/run_all.sh     # solo algunos grupos (a c5 c6 c7 c8 c9 c10)
# Necesita: el workspace compilado (colcon build) y nada más corriendo en ROS_DOMAIN_ID=77.

# (sin set -u: el setup.bash de ROS usa variables sin definir)
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="${1:-$HOME/.ros/rover_arm_sim/resultados_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT"
source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=77
SUMMARY="$OUT/resumen.txt"
: > "$SUMMARY"

start_sim() {   # $1 = nombre, resto = argumentos de launch
  local name=$1; shift
  LOG="$OUT/$name.launch.log"
  timeout -s INT 900 ros2 launch rover_arm_sim arm_sim.launch.py gazebo_gui:=false joystick_window:=false "$@" > "$LOG" 2>&1 &
  SIM_PID=$!
  timeout 120 bash -c "until grep -q 'Initialized from encoders\|Timeout! Missing' '$LOG'; do sleep 0.5; done" || echo "sin arranque" >> "$LOG"
  sleep 2
}

stop_sim() {
  kill -INT $SIM_PID 2>/dev/null
  wait $SIM_PID 2>/dev/null
  for _ in $(seq 1 20); do pgrep -f "practica_brazo.sdf" > /dev/null || break; sleep 0.5; done
  pkill -9 -f "practica_brazo.sdf" 2>/dev/null
  sleep 1
}

run() {         # $1 = archivo de salida, resto = comando
  local f="$OUT/$1"; shift
  (cd "$HERE" && timeout 400 "$@") > "$f" 2>&1
  grep -h "^RESULTADO" "$f" | tee -a "$SUMMARY"
}

want() { [ -z "${ONLY:-}" ] || [[ " $ONLY " == *" $1 "* ]]; }

if want a; then
echo "== criterios 1, 4, 3 (arranque, rechazos/recortes, mando)"
start_sim grupo_a
run c1.txt python3 check_criteria.py c1 --log "$LOG"
run c4.txt python3 check_criteria.py c4
run c3.txt python3 check_criteria.py c3
stop_sim

fi
if want c5; then
echo "== criterio 5 (el brazo sigue al soltar la palanca)"
start_sim grupo_c5
run c5.txt python3 check_criteria.py c5
stop_sim

fi
if want c6; then
echo "== criterio 6 (falla al chocar J2 contra el piso y Reiniciar driver)"
start_sim grupo_c6
run c6.txt python3 check_fault.py
stop_sim

fi
if want c7; then
echo "== criterio 7 (agarre: caja de 5 kg y contenedor con asa)"
start_sim grupo_c7a
run c7_caja.txt python3 check_grasp.py caja_5kg
stop_sim
start_sim grupo_c7b
run c7_asa.txt python3 check_grasp.py contenedor_asa
stop_sim

fi
if want c8; then
echo "== criterio 8 (no atraviesa chasis ni piso)"
start_sim grupo_c8
run c8.txt python3 check_criteria.py c8
stop_sim

fi
if want c9; then
echo "== criterio 9 (cambiar la velocidad de J2 en el YAML)"
CFG2="$OUT/arm_sim_j2_lento.yaml"
sed 's/^      speed_deg_s: 16.6 .*/      speed_deg_s: 8.3                      # PENDIENTE P3 (prueba del criterio 9: mitad)/' \
  "$(ros2 pkg prefix rover_arm_sim)/share/rover_arm_sim/config/arm_sim.yaml" > "$CFG2"
start_sim grupo_c9_normal
run c9_normal.txt python3 check_criteria.py speed
stop_sim
start_sim grupo_c9_lento config:="$CFG2"
run c9_lento.txt python3 check_criteria.py speed
stop_sim
python3 - "$OUT" <<'EOF' | tee -a "$SUMMARY"
import re, sys
out = sys.argv[1]
t = [float(re.search(r'segundos=([\d.]+)', open(f'{out}/{n}.txt').read()).group(1)) for n in ('c9_normal', 'c9_lento')]
exp = [30 / 16.6 + 16.6 / 47.4 + 0.02, 30 / 8.3 + 8.3 / 47.4 + 0.02]
ok = all(abs(a - b) < 0.15 for a, b in zip(t, exp))
print(f'RESULTADO c9 {"OK" if ok else "FALLA"} j2_16.6=%.2fs(esperado %.2f) j2_8.3=%.2fs(esperado %.2f)' % (t[0], exp[0], t[1], exp[1]))
EOF

fi
if want c10; then
echo "== criterio 10 (tiempo real con la ventana del joystick abierta)"
timeout -s INT 900 ros2 launch rover_arm_sim arm_sim.launch.py gazebo_gui:=false joystick_window:=true \
  > "$OUT/grupo_c10.launch.log" 2>&1 &
SIM_PID=$!
LOG="$OUT/grupo_c10.launch.log"
timeout 120 bash -c "until grep -q 'Ventana del joystick lista' '$LOG'; do sleep 0.5; done"
sleep 5
free -m > "$OUT/c10_memoria.txt"
run c10.txt python3 check_criteria.py c10 --samples 200
stop_sim
if [ "${WITH_DISPLAY:-0}" = "1" ]; then
  timeout -s INT 900 ros2 launch rover_arm_sim arm_sim.launch.py gazebo_gui:=true joystick_window:=true \
    > "$OUT/grupo_c10_gui.launch.log" 2>&1 &
  SIM_PID=$!
  LOG="$OUT/grupo_c10_gui.launch.log"
  timeout 150 bash -c "until grep -q 'Ventana del joystick lista' '$LOG'; do sleep 0.5; done"
  sleep 10
  free -m > "$OUT/c10_gui_memoria.txt"
  run c10_gui.txt python3 check_criteria.py c10 --samples 200
  stop_sim
fi
fi
echo "Resultados en $OUT"
cat "$SUMMARY"
