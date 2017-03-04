#!/usr/bin/env python

import tf
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

##################### Params ##################
m = 65  # Mass in grams
#fric_force = 27  # force friction in grams
fric_force = 11  # force friction in grams
fric_force = 14  # Friction real robots
# fric_force = 0
fric_moment = 0.0025  # moment [cm*grams]
# fric_moment = 2.5 # Real robots
fric_moment = 3.1
r = 40.  # radious
################################################3


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

    #### Control
    kp, ki = 2.5, 0
    ## PI Control to get desired accelerations in the environment
    ex = (des_dx - r_dx)
    ey = (des_dy - r_dy)
    ax_global = kp * ex + ki * accum_x
    ay_global = kp * ey + ki * accum_y

    # PI Control for the angular acceleration
    #kp_th, ki_th = -800, 0.
    kp_th, ki_th = .0000002, 0
    kp_th, ki_th = .00004, 0.00000000018  ## REAL robots
    eth = (des_dth - r_dth)
    ath = kp_th * eth + ki_th * accum_th
    # accums
    accum_x += ex
    accum_y += ey
    accum_th += -eth



    if accum_th > 120.:
        accum_th = 120
    if accum_th < -120.:
        accum_th = -120

    #### conver from global to local
    ax = math.cos(theta) * ax_global + math.sin(theta) * ay_global
    ay = -math.sin(theta) * ax_global + math.cos(theta) * ay_global

    # rospy.logwarn([des_dth, r_dth])
    # rospy.logwarn(["a", ax, ay])
    # ax = -0.001
    # ay = 0.001

    ##### Convert accelerations to forces
    a_mag = math.hypot(ax, ay)  # magnitud
    if a_mag>0:
        ux, uy = ax / a_mag, ay / a_mag
    else:
        ux, uy = 0, 0

    # ath = 0.0000

    Fx = m * ax + ux * fric_force
    Fy = m * ay + uy * fric_force
    Mz = m * r**2 * ath
    if Mz>0:
        Mz += fric_moment
    else:
        Mz -= fric_moment

    # Fx = fric_force+1
    # Fx = 0 #m * ax + fric_force
    # Fy = 0
    # Mz = 0.0031

    #rospy.logwarn(["Omni Force", Fx, Fy, Mz, " accels", ax, ay, ath])
    # rospy.logwarn(["Omni Force", Fx, Fy, Mz])
    #rospy.logwarn(["ex", ex, r_dx])
    # rospy.logwarn(["u", ux, uy])

    ## Send package
    twist = Twist()
    twist.linear.x = Fx
    twist.linear.y = Fy
    twist.angular.z = Mz
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
