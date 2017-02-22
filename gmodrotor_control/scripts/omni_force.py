#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import scipy.optimize as optimize


################ Motor vels #####################
import numpy as np

# radious (Cm)
r = 4.
# constants
b = math.sqrt(2) / 2
c = 1. / r

# Matrices
As_i = []

# fx>0, fy>0
A = np.array([[b, -b , b], [b, b, -b], [-c, c, c]])
Ai = np.linalg.inv(A)
As_i.append(Ai)
# fx<0, fy>0
A = np.array([[b, -b , -b], [b, b, -b], [-c, c, -c]])
Ai = np.linalg.inv(A)
As_i.append(Ai)
# fx<0, fy<0
A = np.array([[-b, -b , b], [b, -b, -b], [c, -c, c]])
Ai = np.linalg.inv(A)
As_i.append(Ai)

# fx>0, fy<0
A = np.array([[b, -b , b], [b, -b, -b], [-c, -c, c]])
Ai = np.linalg.inv(A)
As_i.append(Ai)



def compute_motor_forces(fx, fy, mz):
    # Motor velocities
    f1, f2, f3, f4 = 0,0,0,0
    rospy.logwarn(fy)
    # Linear forces
    if fx==0 and fy==0:
        pass
    elif fx>=0 and fy>=0:
        [f1, f2, f4] = np.dot(As_i[0], [fx, fy, 0])
    elif fx<0 and fy>=0:  # fx<0, fy>0
        [f1, f2, f3] = np.dot(As_i[1], [fx, fy, 0])
    elif fx<0 and fy<0:     # fx<0, fy<0
        [f2, f3, f4] = np.dot(As_i[2], [fx, fy, 0])
    elif fx>=0 and fy<0:
        [f1, f3, f4] = np.dot(As_i[3], [fx, fy, 0])


    # Moments
    if mz > 0:
        [f1, f2, f4] = [f1, f2, f4] + np.dot(As_i[0], [0, 0, mz])
    else:
        [f1, f2, f3] = [f1, f2, f3] + np.dot(As_i[1], [0, 0, mz])

    return np.array([f1, f2, f3, f4])


################ End Motor vels #####################





pub1 = None

def callbacallback(twist_msg):
    # desired forces (in grams)
    ax = twist_msg.linear.x
    ay = twist_msg.linear.y
    dth = twist_msg.angular.z


    # Spatial velocites to motor forces
    F = compute_motor_forces(ax, ay, dth)

    # Friction


    ### Convert force to grams
    # PWM thrust
    c1, c2, c3  = -0.670900, 0.193200, 13.065200
    pwm_max, pwm_min = 60000., 10000

    # Forces to PWM
    PWM = (c1 + c2 * np.sqrt(c3 + F)) * pwm_max
    PWM[PWM > pwm_max] = pwm_max  # MAX
    PWM[PWM < pwm_min] = pwm_min  # MIN


    rospy.logwarn(F)
    rospy.logwarn(PWM)
    # Twist package sends PWMs
    twist = Twist()
    twist.linear.x  = PWM[3]  # Motor 4
    twist.linear.y  = PWM[0]  # Motor 3
    twist.linear.z  = PWM[1]  # Motor 2
    twist.angular.z = PWM[2]  # Motor 1


    # publish twist
    pub1.publish(twist)





def listener():
    global pub1
    rospy.init_node('omnicontroller', anonymous=True)

    # Topics toward the robots
    pub1 = rospy.Publisher('cmd_vel', Twist, queue_size=0)
    rospy.Subscriber('omni_force', Twist, callbacallback)
    # pub1 = rospy.Publisher('/crazy01/cmd_vel', Twist, queue_size=0)
    # rospy.Subscriber('/crazy01/omni_force', Twist, callbacallback)


    rospy.spin()









if __name__ == '__main__':
    listener()
