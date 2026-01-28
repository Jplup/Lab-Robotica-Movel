#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from pynput.keyboard import Key,Listener


# estado atual das teclas
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

xPos=0
yPos=0
zDeg=0
cellSize=0.25
idealPos=[0,0]

moveAuto=False

maxTurnSpeed=2
turnKp=0.2
moveKp=4

turnTime=10
timeToGoForward=2
timeToSpin=1.5

currentWalls=[0,0,0]

mouseDirVal=0

sizeList=20
'''distanciasSL=[0.1 for _ in range(sizeList)]
aux=0
distL=0
distanciasS=[0.25 for _ in range(sizeList)]
dist=0'''

distE=0
distF=0
distD=0

precisionD=0.005
precision=1

def getOdom(msg):
    global xPos,yPos,zDeg
    quaternion=msg.pose.pose.orientation
    rotations=euler_from_quaternion([quaternion.x,quaternion.y,quaternion.z,quaternion.w])
    zDeg=(rotations[2]* 180 / math.pi)
    xPos=msg.pose.pose.position.x
    yPos=msg.pose.pose.position.y

'''def scan(msg):
    global aux,distL,distanciasSL,dist,distanciasS
    distanciasSL[aux]=msg.ranges[2]
    distanciasS[aux]=min(0.25,msg.ranges[1])
    distL=sum(distanciasSL)/len(distanciasSL)
    dist=sum(distanciasS)/len(distanciasS)
    aux+=1
    rospy.loginfo("distanciasSL: "+str(distanciasSL))
    rospy.loginfo("distanciasS: " + str(distanciasS))
    if aux>=sizeList: aux=0'''

def scan(msg):
    global distD,distE,distF
    distD=msg.ranges[0]
    distF=msg.ranges[1]
    distE=msg.ranges[2]

def calcAngle():
    delta_x = idealPos[0] - xPos
    delta_y = idealPos[1] - yPos

    target_angle = math.degrees(math.atan2(delta_y, delta_x))

    # Ajuste para garantir que o ângulo esteja entre -180 e 180
    target_angle = (target_angle + 180) % 360 - 180

    # Verificar a diferença de ângulo entre o robô e o target
    angle_diff = target_angle - zDeg

    # Garantir que a diferença de ângulo esteja no intervalo -180 a 180
    if angle_diff > 180:
        angle_diff -= 360
    elif angle_diff < -180:
        angle_diff += 360

    return target_angle

def calcTarget(pub):
    # o robô está virado para esquerda
    rospy.loginfo("Calculando target, zDeg: "+str(zDeg))
    if (zDeg>45 and zDeg<=135):
        idealPos[1] += cellSize
        rospy.loginfo("Ideal pos: " + str(idealPos))
        if abs(zDeg-90): turn(pub,90)
        rospy.loginfo("Ideal: 90, real: " + str(calcAngle()))
        return [xPos,yPos+cellSize]
    # o robô está virado para frente
    elif ((zDeg<=45 or zDeg>315) and zDeg>=0) or (zDeg>-45 and zDeg<45):
        idealPos[0] += cellSize
        rospy.loginfo("Ideal pos: " + str(idealPos))
        if abs(zDeg): turn(pub, 0)
        rospy.loginfo("Ideal: 0, real: " + str(calcAngle()))
        return [xPos+cellSize,yPos]
    # o robô está virado para direita
    elif (zDeg<=315 and zDeg>225) or (zDeg<=-45 and zDeg>-135):
        idealPos[1] -= cellSize
        rospy.loginfo("Ideal pos: " + str(idealPos))
        if abs(zDeg + 90): turn(pub, -90)
        rospy.loginfo("Ideal: -90, real: " + str(calcAngle()))
        return [xPos,yPos-cellSize]
    # o robô está virado para trás
    elif zDeg>135 and zDeg<=225:
        idealPos[0] -= cellSize
        rospy.loginfo("Ideal pos: " + str(idealPos))
        if abs(zDeg - 180): turn(pub, 180)
        rospy.loginfo("Ideal: 180, real: " + str(calcAngle()))

        return [xPos-cellSize,yPos]
    else:
        rospy.loginfo("Deu errado")

def calcTarget2(pub):
    global mouseDirVal
    # o robô está virado para esquerda
    rospy.loginfo("Calculando target, zDeg: "+str(zDeg))
    if (zDeg>45 and zDeg<=135):
        idealPos[1] += cellSize
        rospy.loginfo("Ideal pos: " + str(idealPos))
        if abs(zDeg-90): turn(pub,calcAngle())
        rospy.loginfo("Ideal: 90, real: " + str(calcAngle()))
        mouseDirVal=1
    # o robô está virado para frente
    elif ((zDeg<=45 or zDeg>315) and zDeg>=0) or (zDeg>-45 and zDeg<45):
        idealPos[0] += cellSize
        rospy.loginfo("Ideal pos: " + str(idealPos))
        if abs(zDeg): turn(pub, calcAngle())
        rospy.loginfo("Ideal: 0, real: " + str(calcAngle()))
        mouseDirVal=0
    # o robô está virado para direita
    elif (zDeg<=315 and zDeg>225) or (zDeg<=-45 and zDeg>-135):
        idealPos[1] -= cellSize
        rospy.loginfo("Ideal pos: " + str(idealPos))
        if abs(zDeg + 90): turn(pub, calcAngle())
        rospy.loginfo("Ideal: -90, real: " + str(calcAngle()))
        mouseDirVal=-1
    # o robô está virado para trás
    elif (zDeg>135 and zDeg<=225) or (zDeg<=-135 and zDeg>=-225):
        idealPos[0] -= cellSize
        rospy.loginfo("Ideal pos: " + str(idealPos))
        if abs(zDeg - 180): turn(pub, calcAngle())
        rospy.loginfo("Ideal: 180, real: " + str(calcAngle()))
        mouseDirVal=2
    else:
        rospy.loginfo("Deu errado")
    return idealPos

def turn(pub,targetRotation):
    kp = turnKp
    twist = Twist()
    dif = targetRotation-zDeg
    rospy.loginfo("target and: " + str(targetRotation))
    rate = rospy.Rate(40)
    while abs(dif) > precision:
        dif = targetRotation-zDeg
        #rospy.loginfo("dif ang antes: " + str(dif))
        if dif>=360: dif=dif-360
        if dif<=-360: dif=dif+360
        if dif>180: dif=dif-360
        if dif<-180: dif=dif+360
        angval=max(min(kp * dif,maxTurnSpeed),-maxTurnSpeed)
        twist.angular.z = angval
        pub.publish(twist)
        #rospy.loginfo("dif ang: "+str(dif))
        #rospy.loginfo("Angular: "+str(angval))
        rate.sleep()
    twist.angular.z=0
    pub.publish(twist)
    return

def checkWalls(pub):
    global currentWalls,distD,distE,distF,labirinto,mouseDirVal
    currentWalls=[0,0,0]
    if distD < 0.15: currentWalls[2] = 1
    if distF < 0.15: currentWalls[1] = 1
    if distE < 0.15: currentWalls[0] = 1
    rospy.loginfo("Paredes: "+str(currentWalls))
    rospy.loginfo("distD: " + str(distD))
    rospy.loginfo("distF: " + str(distF))
    rospy.loginfo("distE: " + str(distE))

    if currentWalls[0]==0:
        # esquerda
        move(pub, 1)
        labirinto.updateCoords(mouseDirVal)
        move(pub, 0)


    elif currentWalls[1]==0:
        # frente
        labirinto.updateCoords(mouseDirVal)
        move(pub, 0)

    elif currentWalls[2]==0:
        # direita
        move(pub, -1)
        labirinto.updateCoords(mouseDirVal)
        move(pub, 0)

    else:
        # tras
        move(pub, 2)
        labirinto.updateCoords( mouseDirVal)
        move(pub, 0)


def move(pub,dir):
    rate = rospy.Rate(40)
    # Se 'dir' é -1, então é para virar para a esquerda
    if dir==-1:
        targetRotation=zDeg-90
        turn(pub,targetRotation)
    # Se 'dir' é , então é para seguir em frente
    elif dir==0:
        kp=moveKp
        twist = Twist()
        twist.linear.x=0.2
        targetPosition = calcTarget2(pub)
        dif=math.sqrt(((xPos-targetPosition[0])**2)+((yPos-targetPosition[1])**2))
        difdif=0
        difAnterior=dif
        direcao=1
        rospy.loginfo("target: %s", str(targetPosition))
        rospy.loginfo("pos: %s", str(xPos)+"/"+str(yPos))
        rospy.loginfo("dif: %s", str(dif))
        rospy.loginfo("twist: %s", str(twist.linear))
        while dif > precisionD:
            pub.publish(twist)
            dif = math.sqrt(((xPos - targetPosition[0]) ** 2) + ((yPos - targetPosition[1]) ** 2))
            difdif=dif-difAnterior
            if difdif<0 and direcao==1:
                direcao=-1
            if difdif<0 and direcao==-1:
                direcao=1
            twist.linear.x = min(0.2,kp*dif)*direcao
            #rospy.loginfo("dif: %s", str(dif))
            difAnterior=dif
            rate.sleep()
        twist.linear.x=0
        pub.publish(twist)

    # Se 'dir' é 1, então é para virar para a direita
    elif dir==1:
        targetRotation = zDeg + 90
        turn(pub,targetRotation)
    # Se 'dir' é 2, então é para virar para a trás
    elif dir==2:
        targetRotation = zDeg + 180
        turn(pub,targetRotation)
    else:
        print("Fez tudo errado")

mazeSize=4

class Maze:
    def __init__(self):
        self.mousePos = [0, 0]
        self.mouseCoord = [0, 0]
        self.mouseDir = 0
        self.mazeData = [0 for _ in range(mazeSize)]
        for i in range(mazeSize):
            self.mazeData[i] = [[-1 for _ in range(4)] for _ in range(mazeSize)]
        self.search = False

    def resetProgress(self):
        self.mazeData = [0 for _ in range(len(maze))]
        for i in range(mazeSize):
            self.mazeData[i] = [[-1 for _ in range(4)] for _ in range(mazeSize)]

    def detectWalls(self):
        # cell=maze[self.mouseCoord[0]][self.mouseCoord[1]]
        walls = wallsInMaze[self.mouseCoord[0]][self.mouseCoord[1]]
        if self.mouseDir == -1:
            missingWall = 3
        else:
            missingWall = abs(self.mouseDir)

        for n in range(4):
            if n == missingWall: continue
            self.mazeData[self.mouseCoord[0]][self.mouseCoord[1]][n] = walls[n]

        for n in range(4):
            seenWall = self.mazeData[self.mouseCoord[0]][self.mouseCoord[1]][n]
            # print(" n:",n,"seenWall:",seenWall)
            if seenWall >= 0:
                # print(" Fisrt check")
                if n == 0:
                    leftCoord = [self.mouseCoord[0], self.mouseCoord[1] - 1]
                    # print("  Foi igual a 0: left:",leftCoord)
                    if leftCoord[1] >= 0:
                        self.mazeData[leftCoord[0]][leftCoord[1]][2] = seenWall
                if n == 1:
                    bellowCoord = [self.mouseCoord[0] + 1, self.mouseCoord[1]]
                    # print("  Foi igual a 1: bellow:",bellowCoord)
                    if bellowCoord[0] < len(maze):
                        self.mazeData[bellowCoord[0]][bellowCoord[1]][3] = seenWall
                if n == 2:
                    rightCoord = [self.mouseCoord[0], self.mouseCoord[1] + 1]
                    # print("  Foi igual a 2: right:",rightCoord)
                    if rightCoord[1] < len(maze):
                        self.mazeData[rightCoord[0]][rightCoord[1]][0] = seenWall
                if n == 3:
                    aboveCoord = [self.mouseCoord[0] - 1, self.mouseCoord[1]]
                    # print("  Foi igual a 3: above:",aboveCoord)
                    if aboveCoord[0] >= 0:
                        self.mazeData[aboveCoord[0]][aboveCoord[1]][1] = seenWall

        # print("Missing wall:",missingWall)
        # print("Walls:",walls)
        # print("Cell:",cell)
        # print("MazeData:",self.mazeData[self.mouseCoord[0]][self.mouseCoord[1]])
        # print("All mazeData:",self.mazeData)

        return self.mazeData[self.mouseCoord[0]][self.mouseCoord[1]]

    def getRelativeDirection(self):
        lista = [3, 2, 1, 0]
        novaLista = [0 for _ in range(len(lista))]
        for i in range(len(lista)):
            index = i - self.mouseDir
            if index >= len(lista): index -= len(lista)
            if index < 0: index += len(lista)
            novaLista[i] = lista[index]
        return novaLista

    def getAbsoluteDirection(self):
        lista = [3, 2, 1, 0]
        novaLista = [0 for _ in range(len(lista))]
        for i in range(len(lista)):
            index = i + self.mouseDir
            if index >= len(lista): index -= len(lista)
            if index < 0: index += len(lista)
            novaLista[i] = lista[index]
        return novaLista

    def updateCoords(self,mouseDir):
        global moveAuto,currentWalls
        self.mouseDir=mouseDir

        coordx=int(round(xPos/cellSize))
        coordy=-int(round(yPos/cellSize))
        rospy.loginfo("------------------------------")
        #rospy.loginfo("MouseDirVal: " + str(mouseDir))
        #rospy.loginfo("Move: " + str(move))
        rospy.loginfo("Coord: ("+str(coordx)+","+str(coordy)+")")
        absoluteWalls=self.mazeData[coordx][coordy]
        if mouseDir==0:
            absoluteWalls=[currentWalls[0],currentWalls[1],currentWalls[2],absoluteWalls[3]]
        elif mouseDir==1:
            absoluteWalls = [currentWalls[1], currentWalls[2], absoluteWalls[2],currentWalls[0]]
        elif mouseDir==-1:
            absoluteWalls = [absoluteWalls[0],currentWalls[0], currentWalls[1], currentWalls[2]]
        else:
            absoluteWalls = [currentWalls[2], absoluteWalls[1],currentWalls[0], currentWalls[1]]

        self.mazeData[coordx][coordy] = absoluteWalls
        rospy.loginfo("Absolute walls: " + str(absoluteWalls))
        rospy.loginfo("------------------------------")
        moveAuto=True


    def AutoMove(self):
        walls = self.detectWalls()
        relativeWalls = [walls[i] for i in self.getRelativeDirection()]

    def searchMinus(self):
        for line in self.mazeData:
            for cell in line:
                for value in cell:
                    if value == -1:
                        return True
        return False

def main():
    global moveAuto
    rospy.init_node("left_wall_follower")

    rospy.Subscriber("/scan",LaserScan,scan)
    rospy.Subscriber("/odom",Odometry,getOdom)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    rate = rospy.Rate(20)

    rospy.loginfo("Seguidor de parede esquerda INICIADO!")

    listener = Listener(on_press=on_press, on_release=on_release)
    listener.start()

    # detectar paredes

    while not rospy.is_shutdown():

        if moveAuto:
            checkWalls(pub)
            #moveAuto=False

        # interpreta teclas pressionadas
        if 'w' in keys_pressed:
            move(pub,0)
        if 's' in keys_pressed:
            move(pub,2)
        if 'a' in keys_pressed:
            move(pub,1)
        if 'd' in keys_pressed:
            move(pub,-1)

        if 'x' in keys_pressed:
            moveAuto=True

        rate.sleep()

labirinto=Maze()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass