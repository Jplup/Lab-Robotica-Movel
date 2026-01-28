# Lab-Robotica-Movel

Projeto de simulação e solução de labirintos utilizando ROS e Gazebo para robótica móvel.

# Sistema de Mapeamento:
- É usado um arquivo .txt para gerar o mapa, a partir de uma codificação em que cada número define a presença de certas paredes.
- Cada célula só contém as informações das paredes da esquerda e de cima. O cógido no final preenche automaticamente as paredes do lado direito e embaixo do labirinto. 
- A codificação segue essas instruções:
  - `0`: Sem paredes (na esquerda e em cima)
  - `1`: Apenas parede à esquerda
  - `2`: Apenas parede acima
  - `3`: Parede acima e à esquerda
  O arquivo .txt deve estar dentro do diretório: catkin_ws/src/micro/micromouse_gazebo/maps/

Exemplo mapa codificado:
```
322
131
101
```
Gera esse mapa:
```
┌───┬───┬───┐
│           │
├   ┼───┼   ┤
│   │   │   │
├   ┼   ┼   ┤
│       │   │
└───┴───┴───┘
```
Após modificar ou criar um novo arquivo .txt do mapa, gere o arquivo .world correspondente:

- cd ~/catkin_ws/src/micromouse_gazebo
- python3 scripts/generate_world.py maps/lab1.txt worlds/lab1.world

Para usar arquivos com nomes diferentes:

- cd ~/catkin_ws/src/micromouse_gazebo
- python3 scripts/generate_world.py maps/SEU_MAPA.txt worlds/SEU_MUNDO.world


# Rodar o programa:

- Terminal 1 - ROS Core:

  - roscore

- Terminal 2 - Gazebo:

  - roslaunch micromouse_gazebo lab1.launch

- Terminal 3 - Solver do Labirinto:

  - python catkin_ws/src/micro/Solver/Scripts/flood_fill.py
