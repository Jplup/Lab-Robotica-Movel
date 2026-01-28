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
cellSize=0.2
idealPos=[0,0]

maxTurnSpeed=2
turnKp=0.2
moveKp=4

turnTime=10
timeToGoForward=2
timeToSpin=1.5

sizeList=20
distanciasSL=[0.1 for _ in range(sizeList)]
aux=0
distL=0
distanciasS=[0.25 for _ in range(sizeList)]
dist=0

precisionD=0.005
precision=1

def getOdom(msg):
    global xPos,yPos,zDeg
    quaternion=msg.pose.pose.orientation
    rotations=euler_from_quaternion([quaternion.x,quaternion.y,quaternion.z,quaternion.w])
    zDeg=(rotations[2]* 180 / math.pi)
    xPos=msg.pose.pose.position.x
    yPos=msg.pose.pose.position.y

def scan(msg):
    global aux,distL,distanciasSL,dist,distanciasS
    distanciasSL[aux]=msg.ranges[2]
    distanciasS[aux]=min(0.25,msg.ranges[1])
    distL=sum(distanciasSL)/len(distanciasSL)
    dist=sum(distanciasS)/len(distanciasS)
    aux+=1
    if aux>=sizeList: aux=0

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
    # o robô está virado para esquerda
    rospy.loginfo("Calculando target, zDeg: "+str(zDeg))
    if (zDeg>45 and zDeg<=135):
        idealPos[1] += cellSize
        rospy.loginfo("Ideal pos: " + str(idealPos))
        if abs(zDeg-90): turn(pub,calcAngle())
        rospy.loginfo("Ideal: 90, real: " + str(calcAngle()))
    # o robô está virado para frente
    elif ((zDeg<=45 or zDeg>315) and zDeg>=0) or (zDeg>-45 and zDeg<45):
        idealPos[0] += cellSize
        rospy.loginfo("Ideal pos: " + str(idealPos))
        if abs(zDeg): turn(pub, calcAngle())
        rospy.loginfo("Ideal: 0, real: " + str(calcAngle()))
    # o robô está virado para direita
    elif (zDeg<=315 and zDeg>225) or (zDeg<=-45 and zDeg>-135):
        idealPos[1] -= cellSize
        rospy.loginfo("Ideal pos: " + str(idealPos))
        if abs(zDeg + 90): turn(pub, calcAngle())
        rospy.loginfo("Ideal: -90, real: " + str(calcAngle()))
    # o robô está virado para trás
    elif zDeg>135 and zDeg<=225:
        idealPos[0] -= cellSize
        rospy.loginfo("Ideal pos: " + str(idealPos))
        if abs(zDeg - 180): turn(pub, calcAngle())
        rospy.loginfo("Ideal: 180, real: " + str(calcAngle()))
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
        if dif>=360: dif=dif-360
        if dif<=-360: dif=dif+360
        twist.angular.z = max(min(kp * dif,maxTurnSpeed),-maxTurnSpeed)
        pub.publish(twist)
        #rospy.loginfo("dif ang: "+str(dif))
        rate.sleep()
    twist.angular.z=0
    pub.publish(twist)
    return

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
        rospy.loginfo("target: %s", str(targetPosition))
        rospy.loginfo("pos: %s", str(xPos)+"/"+str(yPos))
        rospy.loginfo("dif: %s", str(dif))
        rospy.loginfo("twist: %s", str(twist.linear))
        while dif > precisionD:
            pub.publish(twist)
            dif = math.sqrt(((xPos - targetPosition[0]) ** 2) + ((yPos - targetPosition[1]) ** 2))
            twist.linear.x = min(0.2,kp*dif)
            #rospy.loginfo("dif: %s", str(dif))
            rate.sleep()
        twist.linear.x=0
        pub.publish(twist)
        return
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

def main():
    rospy.init_node("left_wall_follower")

    rospy.Subscriber("/scan",LaserScan, scan)
    rospy.Subscriber("/odom",Odometry,getOdom)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    rate = rospy.Rate(20)

    rospy.loginfo("Seguidor de parede esquerda INICIADO!")

    listener = Listener(on_press=on_press, on_release=on_release)
    listener.start()

    while not rospy.is_shutdown():

        # interpreta teclas pressionadas
        if 'w' in keys_pressed:
            move(pub,0)
        if 's' in keys_pressed:
            move(pub,2)
        if 'a' in keys_pressed:
            move(pub,1)
        if 'd' in keys_pressed:
            move(pub,-1)

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass