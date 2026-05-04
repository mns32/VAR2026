#!/usr/bin/env bash
#
# run_practica2.sh — Práctica 2 VAR (Carrera de robots) sobre Docker
#
# UBICACIÓN:
#   Coloca este script JUNTO a docker-compose.yml y Dockerfile, es decir:
#       ~/E/var/VAR2026/run_practica2.sh
#
# QUÉ HACE:
#   1) Permite a Docker pintar ventanas X11 (xhost +local:root).
#   2) Levanta el contenedor `var_container` si no está corriendo.
#   3) Compila turtlebot_gazebo_race dentro del contenedor.
#   4) Abre gnome-terminal con 3 pestañas, todas YA dentro del contenedor:
#        · Simulacion → Gazebo + circuito (race.sdf) + Turtlebot3 + bridges
#        · Trabajo    → shell libre con ROS y workspace sourceados
#        · RViz2      → rviz2 con un retardo (espera a Gazebo)
#
# REQUISITO PREVIO (una sola vez):
#   Copia el paquete de la práctica al workspace:
#       cp -r ~/E/var/prac2/turtlebot_gazebo_race  ~/E/var/VAR2026/ros2_ws/
#
# Variables opcionales:
#   SKIP_BUILD=1 ./run_practica2.sh   # no recompila, solo lanza
#   AUTO_DRIVE=1 ./run_practica2.sh   # lanza tambien el wall_follower
#

set -u

# ─── Config ──────────────────────────────────────────────────────────────────
SERVICE="ros2_jazzy"
CONTAINER="var_container"
PKG_NAME="turtlebot_gazebo_race"
CONTROL_PKG_NAME="nav_genetic"
LAUNCH_FILE="create_multi_robot_race.launch.py"
WS_IN_CONTAINER="/home/ros2_ws"
TURTLEBOT3_MODEL="waffle"
RVIZ_DELAY_SECONDS=8

# ─── Localizar la carpeta del docker-compose.yml ─────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [[ ! -f docker-compose.yml ]]; then
    echo "ERROR: no encuentro docker-compose.yml en $SCRIPT_DIR" >&2
    echo "       Coloca este script junto a docker-compose.yml y Dockerfile." >&2
    exit 1
fi

# ─── Detectar 'docker compose' v2 o 'docker-compose' v1 ──────────────────────
if docker compose version >/dev/null 2>&1; then
    DC="docker compose"
elif command -v docker-compose >/dev/null 2>&1; then
    DC="docker-compose"
else
    echo "ERROR: no encuentro docker compose ni docker-compose" >&2
    exit 1
fi

# ─── Comprobar que el paquete del prac2 existe en el workspace ───────────────
if [[ ! -d "$SCRIPT_DIR/ros2_ws/$PKG_NAME" ]]; then
    echo "ERROR: no encuentro $SCRIPT_DIR/ros2_ws/$PKG_NAME" >&2
    echo "" >&2
    echo "       Copia el paquete del zip de la práctica al workspace:" >&2
    echo "         cp -r <ruta>/turtlebot_gazebo_race  $SCRIPT_DIR/ros2_ws/" >&2
    echo "" >&2
    echo "       Por ejemplo:" >&2
    echo "         cp -r ~/E/var/prac2/turtlebot_gazebo_race  $SCRIPT_DIR/ros2_ws/" >&2
    exit 1
fi

# ─── X11 para que Gazebo y RViz puedan abrir ventanas ────────────────────────
if command -v xhost >/dev/null 2>&1; then
    xhost +local:root >/dev/null 2>&1 || true
    echo ">>> xhost: acceso X11 local concedido"
else
    echo "AVISO: xhost no instalado. Si Gazebo/RViz no abren ventana:"
    echo "       sudo apt install x11-xserver-utils"
fi

# ─── Levantar el contenedor si no está corriendo ─────────────────────────────
if docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo ">>> Contenedor $CONTAINER ya está corriendo"
else
    echo ">>> Levantando contenedor $CONTAINER ..."
    $DC up -d "$SERVICE"
    sleep 2  # espera al chown del command de docker-compose
fi

# ─── Compilar el paquete dentro del contenedor ───────────────────────────────
if [[ "${SKIP_BUILD:-0}" != "1" ]]; then
    BUILD_PACKAGES="$PKG_NAME"
    if [[ -d "$SCRIPT_DIR/ros2_ws/$CONTROL_PKG_NAME" ]]; then
        BUILD_PACKAGES="$BUILD_PACKAGES $CONTROL_PKG_NAME"
    fi
    echo ">>> colcon build --packages-select $BUILD_PACKAGES (dentro del contenedor)"
    $DC exec -T "$SERVICE" bash -c "
        set -e
        source /opt/ros/jazzy/setup.bash
        cd $WS_IN_CONTAINER
        colcon build --packages-select $BUILD_PACKAGES --symlink-install
    " || {
        echo "ERROR: el colcon build dentro del contenedor ha fallado" >&2
        exit 1
    }
fi

# ─── Detectar emulador de terminal del host ──────────────────────────────────
TERM_EMU=""
for cand in gnome-terminal konsole xfce4-terminal mate-terminal tilix xterm; do
    if command -v "$cand" >/dev/null 2>&1; then
        TERM_EMU="$cand"
        break
    fi
done
if [[ -z "$TERM_EMU" ]]; then
    echo "ERROR: no encuentro gnome-terminal/konsole/xfce4-terminal/..." >&2
    echo "       sudo apt install gnome-terminal" >&2
    exit 1
fi
echo ">>> Usando terminal: $TERM_EMU"

# ─── Comandos a ejecutar DENTRO del contenedor en cada pestaña ───────────────
ROS_SRC="source /opt/ros/jazzy/setup.bash"
WS_SRC="cd $WS_IN_CONTAINER && source install/setup.bash"
EXPORT_MODEL="export TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
AUTO_DRIVE_ARG="auto_drive:=false"
if [[ "${AUTO_DRIVE:-0}" == "1" ]]; then
    AUTO_DRIVE_ARG="auto_drive:=true"
fi

INSIDE_SIM="$ROS_SRC && $WS_SRC && $EXPORT_MODEL && ros2 launch $PKG_NAME $LAUNCH_FILE $AUTO_DRIVE_ARG"
INSIDE_WORK="$ROS_SRC && $WS_SRC && $EXPORT_MODEL && exec bash"
INSIDE_RVIZ="sleep $RVIZ_DELAY_SECONDS && $ROS_SRC && $WS_SRC && rviz2"

# 'exec bash' al final mantiene la pestaña viva tras Ctrl+C
OUTER_SIM="$DC exec $SERVICE bash -c \"$INSIDE_SIM; exec bash\""
OUTER_WORK="$DC exec $SERVICE bash -c \"$INSIDE_WORK\""
OUTER_RVIZ="$DC exec $SERVICE bash -c \"$INSIDE_RVIZ; exec bash\""

# ─── Lanzar las 3 pestañas ───────────────────────────────────────────────────
if [[ "$TERM_EMU" == "gnome-terminal" ]]; then
    gnome-terminal \
        --tab --title="Simulacion" --working-directory="$SCRIPT_DIR" -- bash -c "$OUTER_SIM" \
        --tab --title="Trabajo"    --working-directory="$SCRIPT_DIR" -- bash -c "$OUTER_WORK" \
        --tab --title="RViz2"      --working-directory="$SCRIPT_DIR" -- bash -c "$OUTER_RVIZ"
else
    echo ">>> $TERM_EMU no soporta el formato de pestañas usado;"
    echo "    abriendo 3 ventanas separadas en su lugar."
    "$TERM_EMU" -e bash -c "$OUTER_SIM"  &
    sleep 0.5
    "$TERM_EMU" -e bash -c "$OUTER_WORK" &
    sleep 0.5
    "$TERM_EMU" -e bash -c "$OUTER_RVIZ" &
fi

echo ">>> Listo. ¡Suerte con la carrera!"
