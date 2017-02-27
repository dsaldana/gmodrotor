#!/usr/bin/env python

import tf
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np

w = 0.116

# w = 0.20
# w= 0.04


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




def get_poses(odoms):
    poses = []
    for odom in odoms:
        # if odom is None:
        #     return [None] * len(odom)
        rx1 = odom.pose.pose.position.x
        ry1 = odom.pose.pose.position.y
        roll1, pitch1, rth = _euler_from_quaterion(odom.pose.pose.orientation)

        poses.append([rx1, ry1, rth])
    return poses

def check_waypoints(odoms, wps, min_dist=.035,  min_ang=.08):
    poses = get_poses(odoms)

    targets_achieved = True
    for (x1, y1, th1), (wx, wy) in zip(poses, wps):
        distance = math.hypot(wy - y1, wx - x1)
        ang_diff = abs(th1 - 0.0)
        target_achieved = distance < min_dist and ang_diff < min_ang

        targets_achieved = targets_achieved and target_achieved
        print distance,

    print ""

    return targets_achieved


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


    # Centroid of the structure
    zc = np.array([.26, 0])


    stage = 1

    while not rospy.is_shutdown():
        # print "stage number", stage

        if odom1 is None or odom2 is None or odom3 is None or odom4 is None:
            continue

        w2 = 2*w * math.sqrt(2)
        odoms = [odom1, odom2, odom3, odom4]

        # robots
        r1 = np.array([odom1.pose.pose.position.x, odom1.pose.pose.position.y])
        r2 = np.array([odom2.pose.pose.position.x, odom2.pose.pose.position.y])
        r3 = np.array([odom3.pose.pose.position.x, odom3.pose.pose.position.y])
        r4 = np.array([odom4.pose.pose.position.x, odom4.pose.pose.position.y])

        # print math.hypot((r3-r2)[0], (r3-r2)[1])

        if stage == 1:  # Stage 1: initial locations

            wp1 = zc
            wp2 = zc + [w2, w2]
            wp3 = zc + [w2, 0]
            wp4 = zc + [-w2,0]
            send_waypoint(pub1, wp1)
            send_waypoint(pub2, wp2)
            send_waypoint(pub3, wp3)
            send_waypoint(pub4, wp4)


            # Check destinations
            achieved = check_waypoints(odoms, [wp1, wp2, wp3, wp4])

            print "stage: ", stage, achieved
            if achieved:
                stage += 1



        elif stage == 2:  # Stage 2a: fist attachment
            # Keep locations
            wp1 = zc
            wp3 = zc + [w2, 0]


            # Move towards 3
            wp2 = r3 + [0, w-0.01]
            # Move towards 1
            wp4 = r1 + [-w, 0]

            send_waypoint(pub1, wp1)
            send_waypoint(pub2, wp2)
            send_waypoint(pub3, wp3)
            send_waypoint(pub4, wp4)

            # Check destinations
            achieved = check_waypoints(odoms, [wp1, wp2, wp3, wp4])

            print "stage: ", stage, achieved
            if achieved:
                stage += 1


        elif stage == 3:  # Stage 2b: fist attachment
            # Keep locations
            wp1 = zc
            wp4 = zc + [-w, 0]


            # Move towards 1
            wp3 = r1 + [w, 0]
            # Move towards nothing
            wp2 = r1 + [w, w + 0.003]


            # print math.hypot((r4-r1)[0], (r4-r1)[1])
            send_waypoint(pub1, wp1)
            send_waypoint(pub2, wp2)
            send_waypoint(pub3, wp3)
            send_waypoint(pub4, wp4)

            achieved = check_waypoints(odoms, [wp1, wp2, wp3, wp4])


            if achieved:
                stage += 1



        rate.sleep()






if __name__ == '__main__':
    listener()
