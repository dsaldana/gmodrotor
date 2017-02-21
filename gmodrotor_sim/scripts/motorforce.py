#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Vector3
import scipy.optimize as optimize


################ Motor vels #####################
import numpy as np

# radious
r = .04
# constants
b = math.sqrt(2) / 2
c = 1. / r

# Matrix
A = [[b, -b, -b, b], [b,b, -b, -b], [-c, c, -c, c]]
A = np.array(A)

# PWM thrust
c1, c2, c3  = -0.670900, 0.193200, 13.065200
pwm_max, pwm_min = 60000., 15000


pub1 = None

def callbacallback(twist_msg):
    m3 = twist_msg.linear.x
    m0 = twist_msg.linear.y
    m1 = twist_msg.linear.z
    m2 = twist_msg.angular.z
    # PWM
    M = np.array([m0, m1, m2, m3])

    # Motor saturation
    M[M>pwm_max] = pwm_max

    # pwm to forces (in grams)
    MF = np.power((M / pwm_max - c1) / c2, 2) - c3
    # forces in kilograms
    MF/=1000
    MF[MF<0] = 0.

    # compute forces
    [fx, fy, mz] = np.dot(A, MF)


    # publish twist
    msg = Wrench()
    force = Vector3()
    torque = Vector3()
    force.x = fx
    force.y = fy
    torque.z = mz

    msg.force = force
    msg.torque = torque
    pub1.publish(msg)




def listener():
    global pub1
    rospy.init_node('motors_force', anonymous=True)

    # Topics toward the robots
    pub1 = rospy.Publisher('morseforce', Wrench, queue_size=0)
    rospy.Subscriber('cmd_vel', Twist, callbacallback)



    rospy.spin()









if __name__ == '__main__':
    listener()
