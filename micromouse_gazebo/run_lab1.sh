#!/bin/bash
set -e
PKG_DIR="$(rospack find micromouse_gazebo)"
MAP="$PKG_DIR/maps/lab1.txt"
WORLD="$PKG_DIR/worlds/lab1.world"

echo "Gerando world a partir do mapa..."
python3 "$PKG_DIR/scripts/generate_world.py" "$MAP" "$WORLD"

echo "Rodando roslaunch..."
roslaunch micromouse_gazebo lab1.launch

