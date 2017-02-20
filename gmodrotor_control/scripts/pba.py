#!/usr/bin/env python

import tf
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist




odom1, odom2, odom3, odom4 =  None, None, None, None




def callbacallback_odom1(odom):
    global odom1
    odom1 = odom

def callbacallback_odom2(odom):
    global odom2
    odom2 = odom

def callbacallback_odom3(odom):
    global odom3
    odom3 = odom

def callbacallback_odom4(odom):
    global odom4
    odom4 = odom


def _euler_from_quaterion(quat):
    quaternion = (quat.x, quat.y, quat.z, quat.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    return roll, pitch, yaw


def _ang_diff(x,y):
    return math.atan2(math.sin(x-y), math.cos(x-y))


def trunk(val, min, max):
    if val > max:
        val = max
    if val < min:
        val = min
    return val




def send_waypoint(pub1, wp1):
    twist = Twist()
    twist.linear.x = wp1[0]
    twist.linear.y = wp1[1]
    pub1.publish(twist)


def listener():
    global pub1, pub2, pub3
    rospy.init_node('pba', anonymous=True)


    # Topics toward the robots
    rospy.Subscriber('/crazyflie01/odom', Odometry, callbacallback_odom1)
    rospy.Subscriber('/crazyflie02/odom', Odometry, callbacallback_odom2)
    rospy.Subscriber('/crazyflie03/odom', Odometry, callbacallback_odom3)
    rospy.Subscriber('/crazyflie04/odom', Odometry, callbacallback_odom4)


    pub1 = rospy.Publisher('/crazyflie01/waypoint', Twist, queue_size=0)
    pub2 = rospy.Publisher('/crazyflie02/waypoint', Twist, queue_size=0)
    pub3 = rospy.Publisher('/crazyflie03/waypoint', Twist, queue_size=0)
    pub4 = rospy.Publisher('/crazyflie04/waypoint', Twist, queue_size=0)



    freq = 10  # 10hz
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():




        # Stage 1: initial locations

        wp1 = [-1.35      , -0.8 ]
        wp3 = [-1.70072496, -0.8       ]
        wp4 = [-2.05144993, -0.8      ]
        wp2 = [-1.35      , -1.15072496]
        # send_waypoint(pub1, wp1)
        # send_waypoint(pub2, wp2)
        # send_waypoint(pub3, wp3)
        # send_waypoint(pub4, wp4)



        # Dependable waypoints




        # Stage 2a: fist attachment
        # Robot locations
        # if odom1 is None or odom2:
        #     rospy.logwarn("No Odometry bla")
        #     rate.sleep()
        #     continue


        rx1 = odom1.pose.pose.position.x
        ry1 = odom1.pose.pose.position.y
        rx2 = odom2.pose.pose.position.x
        ry2 = odom2.pose.pose.position.y
        rx3 = odom3.pose.pose.position.x
        ry3 = odom3.pose.pose.position.y
        rx4 = odom4.pose.pose.position.x
        ry4 = odom4.pose.pose.position.y




        # send_waypoint(pub1, wp1)
        # send_waypoint(pub3, wp3)
        w = 0.11
        #
        # wp2 = [rx1 , ry1 - w]
        # send_waypoint(pub2, wp2)
        #
        # wp4 = [rx3 - w, ry3]
        # send_waypoint(pub4, wp4)



        wp1 = [rx3 + w , ry3]
        send_waypoint(pub1, wp1)

        wp2 = [rx3 + w,  ry3 -w]
        send_waypoint(pub2, wp2)









        # send_waypoint(pub1, wp1)
        # wp2 = wp1
        # wp2[1] -= w
        #
        # send_waypoint(pub2, wp2)


        # Stage 2b final attachment


        #
        #
        #
        # # # Robot pose    # angles from robot 2
        # roll1, pitch1, rth = _euler_from_quaterion(odom1.pose.pose.orientation)
        # rx = odom1.pose.pose.position.x
        # ry = odom1.pose.pose.position.y
        # # Robot velocities
        # rdx = odom1.twist.twist.linear.x
        # rdy = odom1.twist.twist.linear.y
        # rdth = odom1.twist.twist.angular.z
        # #
        # #
        # # des_x = -1.0689
        # # des_y =  -0.399505
        #
        # des_x = waypoint_x
        # des_y = waypoint_y
        # #
        # #
        # # Control
        # twist = Twist()
        #
        # kp = 10
        # ex = (des_x - rx)
        # ey = (des_y - ry)
        #
        # ux = kp * ex
        # uy = kp * ey
        #
        # if math.sqrt(ex**2 + ey**2) < .005:
        #     ux, uy = 0., 0.
        #
        #
        # #vel = 0.02 #not board
        # vel = 0.01 #not board
        # # vel = 0.000001
        # twist.linear.x = trunk(ux, -vel, vel)
        # twist.linear.y = trunk(uy, -vel, vel)
        #
        #
        # #
        # theta_des = 0
        # k_th = 200
        # vth = k_th * (theta_des - rth)
        #
        # # ang = .5  # not board
        # ang = .4
        # twist.angular.z = trunk(vth, -ang, ang)
        # # twist.angular.z = -5



        rate.sleep()






if __name__ == '__main__':
    listener()
