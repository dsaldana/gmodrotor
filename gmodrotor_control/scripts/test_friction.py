#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist



odom1 =  None

def callbacallback_odom1(odom):
    global odom1
    odom1 = odom


"""
Estimate the friction force
"""
def listener():
    global pub1, pub2, pub3
    rospy.init_node('ramping', anonymous=True)

    # Odometry
    rospy.Subscriber('/crazyflie01/odom', Odometry, callbacallback_odom1)
    # Topics toward the robots
    pub1 = rospy.Publisher('/crazyflie01/omni_force', Twist, queue_size=0)

    pwm = 10000

    freq = 1. # .5hz
    rate = rospy.Rate(freq)

    force = 10 # in grams

    while not rospy.is_shutdown():
        if odom1 is None:
            print "sem odometria"
            rospy.logwarn("No Odometry")
            rate.sleep()
            continue

        twist = Twist()


        force +=1
        # twist.linear.x  = force
        twist.angular.z  = force

        print force, odom1.twist.twist.linear.x

        # velocity
        if odom1.twist.twist.linear.x > 0.001:
            print "Friction force = ", force-1
            break


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
        rate.sleep()

    # End
    pub1.publish(Twist())






if __name__ == '__main__':
    listener()
