# Lab-Robotica-Movel

Geração de mapas:
  Cada célula contém as informações das paredes de cima e da esquerda dessa forma:0 -> nenhuma parede / 1 -> só a parede da esquerda / 2 -> só a parede de cima / 3 -> as parede de cima e da esquerda. Exemplo:
# 3233


# caso tenha modificado o .txt de geração de mapa, é preciso rodar o script em python que cria o arquivo no formato usado pelo gazebo(.world), para fazer isso use esses comandos:
cd ~/catkin_ws/src/micromouse_gazebo
python3 scripts/generate_world.py maps/lab1.txt worlds/lab1.world
# Se quiser usar outro .txt ou gerar o .world com outro nome ,é só mudar esses nomes no segundo comando. Se for mudar o nome do .world é preciso mudar o nome nas próximas linhas de comando mas aí tem que mudar

Comandos para rodar o programa:
1 - roscore




# Abrir o gazebo com o labirinto:
roslaunch micromouse_gazebo lab1.launch


# Rodar o solver:
python catkin_ws/src/micro/Solver/Scripts/simpleMove.py 



Projeto de simulação e solução de labirintos utilizando ROS e Gazebo para robótica móvel.

Sistema de Mapeamento:
  É usado um arquivo .txt para gerar o mapa, a partir de uma codificação em que cada número define a presença de certas paredes.
  Cada célula só contém as informações das paredes da esquerda e de cima. O cógido no final preenche automaticamente as paredes do lado direito e embaixo do labirinto. 
  A codificação segue essas instruções:
    0: Sem paredes (na esquerda e em cima)
    1: Apenas parede à esquerda
    2: Apenas parede acima
    3: Parede acima e à esquerda

Exemplo mapa codificado:
3233
Configuração
Geração do Mundo Gazebo

┌───┬───┬───┐
│           │
├   ┼───┼   ┤
│   │   │   │
├   ┼   ┼   ┤
│       │   │
└───┴───┴───┘

Após modificar ou criar um novo arquivo .txt do mapa, gere o arquivo .world correspondente:

cd ~/catkin_ws/src/micromouse_gazebo
python3 scripts/generate_world.py maps/lab1.txt worlds/lab1.world

Para usar arquivos com nomes diferentes:
python3 scripts/generate_world.py maps/SEU_MAPA.txt worlds/SEU_MUNDO.world
Execução
Terminal 1 - ROS Core

roscore
Terminal 2 - Simulação Gazebo

roslaunch micromouse_gazebo lab1.launch
Terminal 3 - Solver do Labirinto

python catkin_ws/src/micro/Solver/Scripts/simpleMove.py
Estrutura de Arquivos

micromouse_gazebo/
├── scripts/
│ └── generate_world.py
├── maps/
│ └── lab1.txt
├── worlds/
│ └── lab1.world
└── launch/
└── lab1.launch

micro/Solver/Scripts/
└── simpleMove.py
Notas

    Certifique-se de que o ROS está instalado e configurado corretamente

    O Gazebo deve estar instalado para visualizar a simulação

    Modificações nos arquivos de mapa exigem a geração do novo arquivo .world

    Para usar diferentes mapas, atualize os comandos e arquivos de launch conforme necessário
