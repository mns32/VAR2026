#!/bin/bash
# Script para lanzar Gazebo + nodo de pointclouds + RViz
# Uso: ./launch_all.sh [HARRIS_SHOT|HARRIS_FPFH]

set -e

# Pipeline por defecto o el que pase el usuario
PIPELINE="${1:-HARRIS_SHOT}"

# Desactivar ROS_LOCALHOST_ONLY para que los topics sean visibles
unset ROS_LOCALHOST_ONLY
export ROS_LOCALHOST_ONLY=0

WS_DIR="$(cd "$(dirname "$0")" && pwd)"

source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

export TURTLEBOT3_MODEL=waffle

echo "=== Lanzando Gazebo + TurtleBot ==="
ros2 launch turtlebot_gazebo_multiple create_multi_robot.launch.py &
PID_GZ=$!

echo "Esperando a que Gazebo arranque..."
sleep 12

echo "=== Lanzando nodo get_pointclouds (pipeline: $PIPELINE) ==="
ros2 run get_pointclouds get_pointclouds_node --ros-args -p pipeline:="$PIPELINE" &
PID_NODE=$!

sleep 2

echo "=== Lanzando RViz ==="
RVIZ_CONFIG="$WS_DIR/src/get_pointclouds/rviz/pointclouds.rviz"
rviz2 -d "$RVIZ_CONFIG" &
PID_RVIZ=$!

echo ""
echo "Todo lanzado."
echo "  Gazebo PID:       $PID_GZ"
echo "  PointClouds PID:  $PID_NODE"
echo "  RViz PID:         $PID_RVIZ"
echo ""
echo "Para mover el robot, abre otro terminal:"
echo "  docker exec -it var_container bash"
echo "  unset ROS_LOCALHOST_ONLY"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo "Pulsa Ctrl+C para cerrar todo."

# Esperar y cerrar todo junto con Ctrl+C
trap "kill $PID_GZ $PID_NODE $PID_RVIZ 2>/dev/null; exit" SIGINT SIGTERM
wait
