# !/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from pynput.keyboard import Key, Listener

# ----------------------------- Variáveis globais ------------------------------

# Parâmetros de simulação
cellSize = 0.25  # tamanho de cada cécula do labirinto
turnKp = 0.2  # Ganho proporcional usado na rotação do robô
moveKp = 4  # Ganho proporcional usado na movimentação do robô
precisionD = 0.005  # Precisão de posição
precision = 1  # Precisão de angulo
mazeSize = 5  # Quantas cécula têm no labirinto
rate = 0  # Rospy update rate

# Parâmetros do robô
maxTurnSpeed = 2  # Velocidade máxima de rotação
maxMoveSpeed = 0.2  # Velocidade máxima

# Sensoriamento
distE = 0  # Distancia até a parede a esquerda do robô (relativo)
distF = 0  # Distancia até a parede a frente do robô (relativo)
distD = 0  # Distancia até a parede a direita do robô (relativo)
currentWalls = [0, 0, 0]  # As paredes que estão sendo observadas pelo robô (esquerda,frente,direita) (relativo)

# Odometria
xPos = 0  # Posição em x do robô
yPos = 0  # Posição em y do robô
zDeg = 0  # Rotação em z do robô em radianos
mouseRotaionIndex = 0  # Rotação em z do robô em coordenadas (1=esquerda,0=frente,-1=direita,2=trás)

# Variáveis auxiliares
moveAuto = False
pub = 0  # O objeto de publicação de mensagens do tipo Twist
targetPos = [0, 0]

# ----------------------------- Coleta de dados ------------------------------

# Coleta de informações sobre teclas apertadas
keys_pressed = set()

def on_press(key):
    try:
        keys_pressed.add(key.char)
    except AttributeError:
        pass

def on_release(key):
    try:
        keys_pressed.remove(key.char)
    except (KeyError, AttributeError):
        pass


# Coleta de dados de Odometria
def getOdom(msg):
    global xPos, yPos, zDeg

    # ----Rotação----
    # Transforma a rotação de quaternion para euler
    quaternion = msg.pose.pose.orientation
    rotations = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    # Força o angulo a ser entre -180 e 180 graus
    #rospy.loginfo("rotations[2]: "+str(rotations[2]))
    zDeg = (math.degrees(rotations[2]) + 180) % 360 - 180
    #rospy.loginfo("zDeg: " + str(zDeg))
    # ----Psição----
    xPos = msg.pose.pose.position.x
    yPos = msg.pose.pose.position.y


# Coleta de dados de sonsores
def scan(msg):
    global distD, distE, distF
    distD = msg.ranges[0]
    distF = msg.ranges[1]
    distE = msg.ranges[2]


# ----------------------------- Funções ------------------------------

# Calcula o ângulo que o robô deve ter para apontar para o objetivo
def CalcAngle():
    delta_x = targetPos[0] - xPos
    delta_y = targetPos[1] - yPos
    targetAngle = math.degrees(math.atan2(delta_y, delta_x))

    # Ajuste para garantir que o ângulo esteja entre -180 e 180
    targetAngle = (targetAngle + 180) % 360 - 180
    # Verificar a diferença de ângulo entre o robô e o target
    angle_diff = targetAngle - zDeg

    # Garantir que a diferença de ângulo esteja no intervalo -180 a 180
    if angle_diff > 180:
        angle_diff -= 360
    elif angle_diff < -180:
        angle_diff += 360

    return targetAngle


# Calcula a posição alvo a partir da rotação e posição do robô e rotaciona
#    ele para apontar para o alvo
def CalcTarget():
    global mouseRotaionIndex

    # O robô está virado para esquerda
    rospy.loginfo("Calculando target, zDeg: " + str(zDeg))
    if (zDeg > 45 and zDeg <= 135):
        targetPos[1] += cellSize
        rospy.loginfo("Ideal pos: " + str(targetPos))
        if abs(zDeg - 90): Turn(CalcAngle())
        rospy.loginfo("Ideal: 90, real: " + str(CalcAngle()))
        mouseRotaionIndex = 1
    # O robô está virado para frente
    elif (zDeg <= 45 and zDeg >= 0) or (zDeg > -45 and zDeg < 0):
        targetPos[0] += cellSize
        rospy.loginfo("Ideal pos: " + str(targetPos))
        if abs(zDeg): Turn(CalcAngle())
        rospy.loginfo("Ideal: 0, real: " + str(CalcAngle()))
        mouseRotaionIndex = 0
    # O robô está virado para direita
    elif (zDeg <= -45 and zDeg > -135):
        targetPos[1] -= cellSize
        rospy.loginfo("Ideal pos: " + str(targetPos))
        if abs(zDeg + 90): Turn(CalcAngle())
        rospy.loginfo("Ideal: -90, real: " + str(CalcAngle()))
        mouseRotaionIndex = -1
    # O robô está virado para trás
    elif zDeg > 135 or zDeg <= -135:
        targetPos[0] -= cellSize
        rospy.loginfo("Ideal pos: " + str(targetPos))
        if abs(zDeg - 180): Turn(CalcAngle())
        rospy.loginfo("Ideal: 180, real: " + str(CalcAngle()))
        mouseRotaionIndex = 2
    # Erro
    else:
        rospy.loginfo("Deu errado")
    return targetPos


# Rotaciona o robô até um angulo passado
def Turn(targetRotation):
    twist = Twist()

    # Garante que o angulo alvo está entre -180 e 180 graus
    targetRotation = (targetRotation + 180) % 360 - 180
    dif = targetRotation - zDeg
    # Garante que a rotação é para a direção mais curta
    if dif > 180: dif -= 360
    if dif < -180: dif += 360

    rospy.loginfo("target and: " + str(targetRotation))
    while abs(dif) > precision:
        dif = targetRotation - zDeg
        # rospy.loginfo("dif ang antes: "+str(dif))
        # Garante que a rotação é para a direção mais curta
        if dif > 180: dif -= 360
        if dif < -180: dif += 360

        saturatedTurnSpeed = max(min(turnKp * dif, maxTurnSpeed), -maxTurnSpeed)
        twist.angular.z = saturatedTurnSpeed
        pub.publish(twist)
        # rospy.loginfo("dif ang: "+str(dif))
        # rospy.loginfo("Angular: "+str(saturatedTurnSpeed))
        rate.sleep()

    twist.angular.z = 0
    pub.publish(twist)
    return


# Transforma as informações de sensores em quais paredes estão presentes
def CheckWalls():
    global currentWalls, labirinto, mouseRotaionIndex
    currentWalls = [0, 0, 0]

    # Faz a conversão
    if distD < 0.15: currentWalls[2] = 1
    if distF < 0.15: currentWalls[1] = 1
    if distE < 0.15: currentWalls[0] = 1
    rospy.loginfo("distD: " + str(distD))
    rospy.loginfo("distF: " + str(distF))
    rospy.loginfo("distE: " + str(distE))
    rospy.loginfo("Paredes: " + str(currentWalls))


'''# Decide qual movimento fazer com base nas paredes presentes
def MoveDecision():
    global mouseRotaionIndex,labirinto

    # Toma a decisão de movimento
    if currentWalls[0]==0:
        # Virar para a esquerda 
        Move(1)
        labirinto.updateCoords(mouseRotaionIndex)
    elif currentWalls[1]==0:
        # Mover para frente *
        labirinto.updateCoords(mouseRotaionIndex)
    elif currentWalls[2]==0:
        # Virar para a direita
        Move(-1)
        labirinto.updateCoords(mouseRotaionIndex)
    else:
        # Virar para a tras
        Move(2)
        labirinto.updateCoords( mouseRotaionIndex)

    # Mover para frente
    Move(0)'''


def Move(dir):
    # Se 'dir' é 1, então é para virar para a esquerda
    if dir == 1:
        targetRotation = zDeg + 90
        Turn(targetRotation)
    # Se 'dir' é , então é para seguir em frente
    elif dir == 0:
        twist = Twist()
        targetPosition = CalcTarget()
        dif = math.sqrt(((xPos - targetPosition[0]) ** 2) + ((yPos - targetPosition[1]) ** 2))
        difAnterior = dif
        dirMult = 1
        rospy.loginfo("target: " + str(targetPosition))
        rospy.loginfo("pos: " + str(xPos) + "/" + str(yPos))
        rospy.loginfo("dif: " + str(dif))
        rospy.loginfo("twist: " + str(twist.linear))
        while dif > precisionD:
            dif = math.sqrt(((xPos - targetPosition[0]) ** 2) + ((yPos - targetPosition[1]) ** 2))
            difdif = dif - difAnterior

            # Se a distancia até o alvo começou a crescer -> inverter direção
            if difdif < 0 and dirMult == 1:
                dirMult = -1
            if difdif < 0 and dirMult == -1:
                dirMult = 1

            twist.linear.x = max(min(maxMoveSpeed, moveKp * dif * dirMult), -maxMoveSpeed)
            # rospy.loginfo("dif: %s", str(dif))
            difAnterior = dif

            pub.publish(twist)
            rate.sleep()
        twist.linear.x = 0
        pub.publish(twist)

    # Se 'dir' é -1, então é para virar para a direita
    elif dir == -1:
        targetRotation = zDeg - 90
        Turn(targetRotation)
    # Se 'dir' é 2, então é para virar para a trás
    elif dir == 2:
        targetRotation = zDeg + 180
        Turn(targetRotation)
    else:
        print("Fez tudo errado")


class Maze:
    def __init__(self):
        self.mousePos = [0, 0]
        self.mouseCoord = [0, 0]
        self.mazeData = [0 for _ in range(mazeSize)]
        for i in range(mazeSize):
            self.mazeData[i] = [[-1 for _ in range(4)] for _ in range(mazeSize)]
        self.search = False

    def recalculate_distances(self):
        """Recalcula distâncias usando Flood Fill baseado nas paredes conhecidas"""
        # Células objetivo (centro 2x2)
        goal_cells = []
        center = mazeSize // 2

        if mazeSize % 2 == 0:
            goal_cells = [(center - 1, center - 1), (center - 1, center),
                          (center, center - 1), (center, center)]
        else:
            goal_cells = [(center, center)]

        # Inicializa nova matriz de distâncias com valor alto
        new_dist = [[9999 for _ in range(mazeSize)] for _ in range(mazeSize)]

        # Fila manual para BFS (sem collections.deque)
        queue = []
        queue_index = 0  # Índice para simular pop(0)

        # Inicializa células objetivo com distância 0
        for gx, gy in goal_cells:
            new_dist[gx][gy] = 0
            queue.append((gx, gy))

        debugCalc=False

        if debugCalc: rospy.loginfo("mazedata 2: " + str(self.mazeData))
        # BFS para propagar distâncias
        while queue_index < len(queue):
            if debugCalc: rospy.loginfo(" qi: " + str(queue_index))
            if debugCalc: rospy.loginfo(" len(q): " + str(len(queue)))
            x, y = queue[queue_index]
            queue_index += 1
            current_dist = new_dist[x][y]
            if debugCalc: rospy.loginfo(" qi: " + str(queue_index))
            if debugCalc: rospy.loginfo(" x, y: "+str(x)+", "+str(y))
            if debugCalc: rospy.loginfo(" current dist: " + str(current_dist))

            # Direções: 0=esquerda, 1=frente, 2=direita, 3=trás
            # Em coordenadas absolutas: 0=oeste, 1=norte, 2=leste, 3=sul
            directions = [
                (0, -1, 0),  # esquerda/oeste
                (1, 0, 1),  # frente/norte
                (0, 1, 2),  # direita/leste
                (-1, 0, 3)  # trás/sul
            ]

            for dx, dy, wall_index in directions:
                nx, ny = x + dx, y + dy
                if debugCalc: rospy.loginfo("   nx, ny, wi: " + str(nx)+", "+str(ny)+", "+str(wall_index))
                # Verifica se está dentro do labirinto
                if 0 <= nx < mazeSize and 0 <= ny < mazeSize:
                    # Verifica se não há parede conhecida bloqueando
                    # Se mazeData[x][y][wall_index] = 0 → não tem parede
                    # Se = 1 → tem parede
                    # Se = -1 → não sabe ainda
                    if debugCalc: rospy.loginfo("     Passou no if: " + str())
                    if debugCalc: rospy.loginfo("     mazeDataNoPonto: " + str(self.mazeData[x][y][wall_index]))
                    # Só considera caminho se não tem parede ou é -1 (mão sabemos)
                    if self.mazeData[x][y][wall_index] == 0 or self.mazeData[x][y][wall_index] == -1:
                        if debugCalc: rospy.loginfo("        passou no if2, newDist: " + str(new_dist[nx][ny]))
                        if new_dist[nx][ny] == 9999:
                            new_dist[nx][ny] = current_dist + 1
                            if debugCalc: rospy.loginfo("           passou no if3, newDist: " + str(new_dist[nx][ny]))
                            queue.append((nx, ny))
                            if debugCalc: rospy.loginfo("           new queue: " + str(queue))

        # Atualiza a matriz de distâncias
        rospy.loginfo("Distances: "+str(new_dist))
        self.distances = new_dist

    def get_best_move(self):
        """Retorna a melhor direção baseada no Flood Fill"""
        # Mapeia rotação do mouse para direções absolutas
        # mouseRotaionIndex: 0=frente, 1=esquerda, -1=direita, 2=trás
        # Direções absolutas no mazeData: 0=oeste, 1=norte, 2=leste, 3=sul

        # Procura o vizinho acessível com menor distância
        min_dist = 9999
        best_dir = None

        directions = [
            (-1, 0, 0),  # esquerda/oeste
            (0, 1, 1),  # frente/norte
            (1, 0, 2),  # direita/leste
            (0, -1, 3)  # trás/sul
        ]
        directions = [
            (0, -1, 0),  # esquerda/oeste
            (1, 0, 1),  # frente/norte
            (0, 1, 2),  # direita/leste
            (-1, 0, 3)  # trás/sul
        ]
        rospy.loginfo("self coords: " + str(self.coordx) + ", " + str(self.coordy))
        rospy.loginfo("mazeData: " + str(self.mazeData))
        for dx, dy, wall_index in directions:
            newX, newY = int(self.coordx + dx), int(self.coordy + dy)
            rospy.loginfo("newX, newY: "+str(newX)+", "+str(newY))
            rospy.loginfo("Wall index: "+str(wall_index))
            # Checa se a nova célula está dentro do labirinto e não tem parede até ela
            if 0 <= newX < mazeSize and 0 <= newY < mazeSize and self.mazeData[self.coordx][self.coordy][wall_index]==0:
                rospy.loginfo("   Passou no if")
                if self.distances[newX][newY] < min_dist:
                    rospy.loginfo("      É menor")
                    best_dir = wall_index
                    min_dist = self.distances[newX][newY]

        rospy.loginfo("Best wall: "+str(best_dir))
        rospy.loginfo("Mouse dir index: "+str(mouseRotaionIndex))

        # Se o mouse está apontado para a esquerda
        if mouseRotaionIndex == 2:
            # E a melhor direção é para a esquerda -> seguir em frente
            if best_dir == 0: return 0
            # E a melhor direção é para a frente -> virar para a direita relativa
            if best_dir == 1: return -1
            # E a melhor direção é para a direita -> virar para trás relativa
            if best_dir == 2: return 2
            # E a melhor direção é para trás -> virar para a esquerda relativa
            if best_dir == 3: return 1
        # Se o mouse está apontado para frente
        elif mouseRotaionIndex == 1:
            # E a melhor direção é para a esquerda -> virar para a esquerda relativa
            if best_dir == 0: return 1
            # E a melhor direção é para a frente -> seguir em frente
            if best_dir == 1: return 0
            # E a melhor direção é para a direita -> virar para a direita relativa
            if best_dir == 2: return -1
            # E a melhor direção é para trás -> virar para trás relativa
            if best_dir == 3: return 2
        # Se o mouse está apontado para a direita
        elif mouseRotaionIndex == 0:
            # E a melhor direção é para a esquerda -> virar para trás relativa
            if best_dir == 0: return 2
            # E a melhor direção é para a frente ->  virar para a esquerda relativa
            if best_dir == 1: return 1
            # E a melhor direção é para a direita -> seguir em frente
            if best_dir == 2: return 0
            # E a melhor direção é para trás -> virar para a direita relativa
            if best_dir == 3: return -1
        # Se o mouse está apontado para trás
        else:
            # E a melhor direção é para a esquerda -> virar para a direita relativa
            if best_dir == 0: return -1
            # E a melhor direção é para a frente -> virar para trás relativa
            if best_dir == 1: return 2
            # E a melhor direção é para a direita -> virar para a esquerda relativa
            if best_dir == 2: return 1
            # E a melhor direção é para trás -> seguir em frente
            if best_dir == 3: return 0

    # Faz o update das informações de paredes
    def updateCoords(self):
        coordx = int(round(xPos / cellSize))
        coordy = -int(round(yPos / cellSize))
        self.coordx = coordx
        self.coordy = coordy

        # Faz o update da própria célula
        absoluteWalls = self.mazeData[coordx][coordy]
        if mouseRotaionIndex == 0:
            absoluteWalls = [absoluteWalls[0], currentWalls[0], currentWalls[1], currentWalls[2]]
        elif mouseRotaionIndex == 1:
            absoluteWalls = [currentWalls[0], currentWalls[1], currentWalls[2], absoluteWalls[3]]
        elif mouseRotaionIndex == -1:
            absoluteWalls = [currentWalls[2], absoluteWalls[1], currentWalls[0], currentWalls[1]]
        else:
            absoluteWalls = [currentWalls[1], currentWalls[2], absoluteWalls[2], currentWalls[0]]


        self.mazeData[coordx][coordy] = absoluteWalls

        # Faz o update de células adjacentes
        for n in range(4):
            seenWall = self.mazeData[coordx][coordy][n]
            # Se tem a informação da parede (diferente de -1)
            if seenWall >= 0:
                # Se foi vista uma parede na esquerda
                if n == 0:
                    leftCoord = [coordx - 1, coordy]
                    # Checa para ver se existe cécula na esquerda
                    if leftCoord[0] >= 0:
                        self.mazeData[leftCoord[0]][leftCoord[1]][2] = seenWall
                # Se foi vista uma parede na frente
                if n == 1:
                    aboveCoord = [coordx, coordy + 1]
                    # Checa para ver se existe cécula na frente
                    if aboveCoord[1] < mazeSize:
                        self.mazeData[aboveCoord[0]][aboveCoord[1]][3] = seenWall
                # Se foi vista uma parede na direita
                if n == 2:
                    rightCoord = [coordx + 1, coordy]
                    # Checa para ver se existe cécula na direita
                    if rightCoord[0] < mazeSize:
                        self.mazeData[rightCoord[0]][rightCoord[1]][0] = seenWall
                # Se foi vista uma parede atrás
                if n == 3:
                    bellowCoord = [coordx, coordy - 1]
                    # Checa para ver se existe cécula atrás
                    if bellowCoord[1] >= 0:
                        self.mazeData[bellowCoord[0]][bellowCoord[1]][1] = seenWall
    def GetNextMove(self):
        self.updateCoords()
        self.recalculate_distances()
        moove = self.get_best_move()
        rospy.loginfo("Best move: "+str(moove))
        return moove

labirinto = Maze()

def main():
    global moveAuto, pub, rate

    # Rospy init
    rospy.init_node("micromouse")
    rospy.Subscriber("/scan", LaserScan, scan)
    rospy.Subscriber("/odom", Odometry, getOdom)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    listener = Listener(on_press=on_press, on_release=on_release)
    listener.start()
    rate = rospy.Rate(40)

    rospy.loginfo("Script iniciado")

    # Loop principal
    while not rospy.is_shutdown():

        if moveAuto:
            CheckWalls()
            nextMove = labirinto.GetNextMove()
            # Mover para a esquerda relativa
            if nextMove == 1:
                Move(1)
            # Mover para a esquerda relativa
            elif nextMove == 0:
                pass
            # Mover para a esquerda relativa
            elif nextMove == -1:
                Move(-1)
            # Mover para a esquerda relativa
            else:
                Move(2)

            Move(0)

        # Interpreta teclas pressionadas
        if 'w' in keys_pressed:
            Move(0)
        if 's' in keys_pressed:
            Move(2)
        if 'a' in keys_pressed:
            Move(1)
        if 'd' in keys_pressed:
            Move(-1)
        if 'x' in keys_pressed:
            moveAuto = True

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
