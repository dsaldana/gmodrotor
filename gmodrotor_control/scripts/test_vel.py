#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist









def listener():
    global pub1, pub2, pub3
    rospy.init_node('ramping', anonymous=True)

    # Topics toward the robots
    pub1 = rospy.Publisher('/crazyflie01/omni_vel', Twist, queue_size=0)

    i = 0
    pwm = 10000

    freq = 10 # 10hz
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():

        twist = Twist()

        twist.linear.y  = 30
        # twist.linear.y  = 50
        # itera = i / freq % 10
        # if itera < 2.5:
        #     twist.linear.x  = pwm
        # elif itera < 5:
        #     twist.linear.y  = pwm
        # elif itera < 7.5:
        #     twist.linear.z  = pwm
        # elif itera < 10:
        #     twist.angular.z  = pwm

        pub1.publish(twist)
        i+=1
        rate.sleep()






if __name__ == '__main__':
    listener()
