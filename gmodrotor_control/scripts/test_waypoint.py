#!/usr/bin/env python

import tf
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist




odom1, odom2, odom3 =  None, None, None




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


def trunk(val, min, max):
    if val > max:
        val = max
    if val < min:
        val = min
    return val




def listener():
    global pub1, pub2, pub3
    rospy.init_node('testwaypoint', anonymous=True)



    # Topics toward the robots
    pub1 = rospy.Publisher('omni_vel', Twist, queue_size=0)
    rospy.Subscriber('odom', Odometry, callbacallback_odom1)
    # pub1 = rospy.Publisher('omni_vel', Twist, queue_size=0)
    # rospy.Subscriber('odom', Odometry, callbacallback_odom1)

    kpi_x,kpi_y = 0, 0

    ex_old = 0
    ey_old = 0

    freq = 10 # 10hz
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():

        if odom1 is None:
            print "sem odometria"
            rospy.logwarn("No Odometry")
            rate.sleep()
            continue



        # # Robot pose    # angles from robot 2
        roll1, pitch1, rth = _euler_from_quaterion(odom1.pose.pose.orientation)
        rx = odom1.pose.pose.position.x
        ry = odom1.pose.pose.position.y
        # Robot velocities
        rdx = odom1.twist.twist.linear.x
        rdy = odom1.twist.twist.linear.y
        rdth = odom1.twist.twist.angular.z
        #

        des_x, des_y = 1,1


        # Control
        twist = Twist()

        kp = 10
        ex = (des_x - rx)
        ey = (des_y - ry)

        ux = kp * ex
        uy = kp * ey

        if math.sqrt(ex**2 + ey**2) < .01:
            ux, uy = 0., 0.


        vel = 0.1  #not board
        twist.linear.x = trunk(ux, -vel, vel)
        twist.linear.y = trunk(uy, -vel, vel)


        #
        theta_des = 0
        k_th = 200
        eth = theta_des - rth
        # eth = _ang_diff(theta_des, rth)
        vth = k_th * eth

        ang = .2
        twist.angular.z = trunk(vth, -ang, ang)


        pub1.publish(twist)
        rate.sleep()






if __name__ == '__main__':
    listener()
