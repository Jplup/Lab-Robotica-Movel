#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def moveMouse():
    rospy.init_node('moveMouse')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    msg = Twist()

    msg.linear.x = 0.1
    msg.angular.z = 0.0
    t_start = rospy.Time.now().to_sec()

    while rospy.Time.now().to_sec() - t_start < 5:
        pub.publish(msg)
        rate.sleep()


    msg.linear.x = 0
    msg.angular.z = -0.5
    t_start = rospy.Time.now().to_sec()

    while rospy.Time.now().to_sec() - t_start < 3:
        pub.publish(msg)
        rate.sleep()

    msg.linear.x = 0.0
    msg.angular.z = 0.0
    pub.publish(msg)

if __name__ == '__main__':
    try:
        moveMouse()
    except rospy.ROSInterruptException:
        pass
