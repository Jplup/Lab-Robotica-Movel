#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from pynput.keyboard import Key,Listener
from sensor_msgs.msg import LaserScan

# velocidades
LIN_VEL = 0.15  # m/s
ANG_VEL = 1.0   # rad/s

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

distanciasSL=[0.1 for _ in range(10)]
aux=0
distL=0
distanciasS=[0.1 for _ in range(10)]
dist=0

def scan(msg):
    global aux,distL,distanciasSL,dist,distanciasS
    distanciasSL[aux]=msg.ranges[2]
    distanciasS[aux]=min(0.25,msg.ranges[1])
    distL=sum(distanciasSL)/len(distanciasSL)
    dist=sum(distanciasS)/len(distanciasS)
    aux+=1
    if aux>=9: aux=0

def main():
    global dist,distL
    rospy.init_node('teleop_pynput')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(20)
    twist = Twist()
    twistTeste = Twist()

    rospy.Subscriber("/scan", LaserScan, scan)
    pub2 = rospy.Publisher("/teste", Twist)

    listener = Listener(on_press=on_press, on_release=on_release)
    listener.start()

    rospy.loginfo("Controle com teclas W A S D — pressione Ctrl+C para sair")

    while not rospy.is_shutdown():
        # reset velocidades
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        twistTeste.linear.x=distL
        twistTeste.linear.y=dist

        # interpreta teclas pressionadas
        if 'w' in keys_pressed:
            twist.linear.x += LIN_VEL
        if 's' in keys_pressed:
            twist.linear.x -= LIN_VEL
        if 'a' in keys_pressed:
            twist.angular.z += ANG_VEL
        if 'd' in keys_pressed:
            twist.angular.z -= ANG_VEL

        pub2.publish(twistTeste)

        pub.publish(twist)
        rate.sleep()

    listener.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
