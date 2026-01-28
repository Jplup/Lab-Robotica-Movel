#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Distância mínima segura (em metros)
MIN_DIST = 0.05

# Variáveis globais para armazenar leituras
front = left = right = 1.0

def scan_front(msg):
    global front
    front = min(msg.ranges)

def scan_left(msg):
    global left
    left = min(msg.ranges)

def scan_right(msg):
    global right
    right = min(msg.ranges)

def main():
    rospy.init_node('avoid_walls_simple')
    rospy.Subscriber('/scan', LaserScan, scan_front)
    rospy.Subscriber('/scanlf', LaserScan, scan_left)
    rospy.Subscriber('/scanrf', LaserScan, scan_right)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)
    move = Twist()

    rospy.loginfo("Evitação de paredes iniciada!")

    while not rospy.is_shutdown():
        # lógica de desvio simples
        if front < MIN_DIST:
            # parede na frente → gira para o lado com mais espaço
            if left > right:
                move.linear.x = 0.0
                move.angular.z = 0.5
            else:
                move.linear.x = 0.0
                move.angular.z = -0.5
        elif left < MIN_DIST:
            # muito perto da parede esquerda → vira ligeiramente à direita
            move.linear.x = 0.05
            move.angular.z = -0.3
        elif right < MIN_DIST:
            # muito perto da parede direita → vira ligeiramente à esquerda
            move.linear.x = 0.05
            move.angular.z = 0.3
        else:
            # caminho livre
            move.linear.x = 0.1
            move.angular.z = 0.0

        pub.publish(move)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
