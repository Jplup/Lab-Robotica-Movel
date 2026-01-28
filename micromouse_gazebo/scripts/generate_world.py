#!/usr/bin/env python3
import sys

# Uso:
# python3 generate_world.py maps/lab1.txt worlds/lab1.world [cell_size]

if len(sys.argv) < 3:
    print("Uso: generate_world.py <map_file> <output_world> [cell_size]")
    sys.exit(1)

map_file = sys.argv[1]
output_file = sys.argv[2]
cell_size = float(sys.argv[3]) if len(sys.argv) > 3 else 0.25
wall_thickness = 0.02  # 2 cm
wall_height = 0.10     # 10 cm

# Função utilitária
def add_wall(x, y, length, orientation, wall_id):
    """Gera XML de uma parede no formato SDF."""
    if orientation == "horizontal":
        size_x, size_y = length, wall_thickness
    else:  # vertical
        size_x, size_y = wall_thickness, length

    return f"""
    <model name="wall_{wall_id}">
      <static>true</static>
      <pose>{x} {y} {wall_height/2} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{size_x} {size_y} {wall_height}</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{size_x} {size_y} {wall_height}</size></box>
          </geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient></material>
        </visual>
      </link>
    </model>
    """

# Lê o mapa
with open(map_file, "r") as f:
    grid = [list(line.strip()) for line in f.readlines() if line.strip()]

rows = len(grid)
cols = len(grid[0])
print(f"Mapa: {rows}x{cols}")

# Começa o arquivo SDF
world = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="micromouse_world">
  
    <include>
      <uri>model://ground_plane</uri>
    </include>
"""

wall_id = 0

# Gera paredes internas
for i in range(rows):
    for j in range(cols):
        val = int(grid[i][j])
        x = j * cell_size
        y = -i * cell_size  # invertido pra alinhar visualmente

        # Parede à esquerda
        if val in (1, 3):
            wx = x - cell_size / 2
            wy = y
            wall_id += 1
            world += add_wall(wx, wy, cell_size, "vertical", wall_id)

        # Parede em cima
        if val in (2, 3):
            wx = x
            wy = y + cell_size / 2
            wall_id += 1
            world += add_wall(wx, wy, cell_size, "horizontal", wall_id)

# Paredes da borda direita e inferior
for i in range(rows):
    # Parede direita
    x = cols * cell_size - cell_size / 2
    y = -i * cell_size
    wall_id += 1
    world += add_wall(x, y, cell_size, "vertical", wall_id)

for j in range(cols):
    # Parede inferior
    x = j * cell_size
    y = -rows * cell_size + cell_size / 2
    wall_id += 1
    world += add_wall(x, y, cell_size, "horizontal", wall_id)

# Fecha o arquivo
world += """
  </world>
</sdf>
"""

with open(output_file, "w") as f:
    f.write(world)

print(f"World gerado: {output_file} ({rows}x{cols} células, {wall_id} paredes)")
