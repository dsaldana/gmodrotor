#!/usr/bin/env python

import tf
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist




odom1 =  None
accum_x, accum_y, accum_th = 0,0,0


def callbacallback_odom1(odom):
    global odom1
    odom1 = odom



def _euler_from_quaterion(quat):
    quaternion = (quat.x, quat.y, quat.z, quat.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    return roll, pitch, yaw


def _ang_diff(x,y):
    return math.atan2(math.sin(x-y), math.cos(x-y))


def trunk(val, max):
    if val > max:
        val = max
    if val < -max:
        val = -max
    return val

def _euler_from_quaterion(quat):
    quaternion = (quat.x, quat.y, quat.z, quat.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    return roll, pitch, yaw



def callbacallback_vel(twist_msg):
    global accum_x, accum_y, accum_th

    if odom1 is None:
        rospy.logwarn("No Odometry")
        return

    #### Robot pose
    # Robot pose    # angles from robot 2
    roll1, pitch1, theta = _euler_from_quaterion(odom1.pose.pose.orientation)
    # Robot velocities
    r_dx = odom1.twist.twist.linear.x
    r_dy = odom1.twist.twist.linear.y
    r_dth = odom1.twist.twist.angular.z

    ### Desired velocities
    des_dx = twist_msg.linear.x
    des_dy = twist_msg.linear.y
    des_dth = twist_msg.angular.z





    kp, ki = 1800, 40
    # kp, ki = 2200, 40
    ## PD Control
    ex = (des_dx - r_dx)
    ey = (des_dy - r_dy)
    ax_global = kp * ex + ki * accum_x
    ay_global = kp * ey + ki * accum_y


    kp_th, ki_th = -800, 0.
    eth = (des_dth - r_dth)
    ath = kp_th * eth + ki_th * accum_th



    # conver from global to local
    theta = -theta
    ax = math.cos(theta) * ax_global - math.sin(theta) * ay_global
    ay = math.sin(theta) * ax_global + math.cos(theta) * ay_global

    # accums
    accum_x += ex
    accum_y += ey
    accum_th += eth

    ## Send package
    twist = Twist()
    twist.linear.x  = ax
    twist.linear.y  = ay
    twist.angular.z  =  ath
    print des_dx, des_dy, des_dth
    # publish twist
    pub1.publish(twist)

def listener():
    global pub1
    rospy.init_node('vel_controller', anonymous=True)

    # Topics toward the robots
    # pub1 = rospy.Publisher('/crazyflie02/omni_force', Twist, queue_size=0)
    # rospy.Subscriber('/crazyflie02/odom', Odometry, callbacallback_odom1)
    # rospy.Subscriber('/crazyflie02/omni_vel', Twist, callbacallback_vel)
    pub1 = rospy.Publisher('omni_force', Twist, queue_size=0)
    rospy.Subscriber('odom', Odometry, callbacallback_odom1)
    rospy.Subscriber('omni_vel', Twist, callbacallback_vel)

    rospy.spin()







if __name__ == '__main__':
    listener()
