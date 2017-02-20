#!/usr/bin/env python



import rospy
import tf
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from quadrotor_msgs.msg import PositionCommand
from quadrotor_msgs.msg import SO3Command


pub1, pub2, pub3 = None, None, None
twist2 = None
odom1, odom2, odom3 =  None, None, None


def _mod_twist(twist2, f1):
    twist = Twist()
    twist.linear.x  = twist2.linear.x
    twist.linear.y  = twist2.linear.y
    twist.angular.z  = twist2.angular.z
    #twist.angular.z  = 0

    # Variable thrust
    twist.linear.z  = f1 +10000
    return twist



def callbacallback_trajectory(so3cmd):
    #if twist1 is None or twist2 is None or twist3 is None:
    if twist2 is None:
        return

    # angles from robot 2
    roll2, pitch2, yaw2 = _euler_from_quaterion(odom2.pose.pose.orientation)
    # Angular velocities from robot 2
    dang2 = odom2.twist.twist.angular  # angular velocities
    droll2, dpitch2, dyaw2 = dang2.x, dang2.y, dang2.z

    # desired angles
    roll_des, pitch_des, yaw_des =  _euler_from_quaterion(so3cmd.orientation)
    dangs_des = so3cmd.angular_velocity  # desire angular velocities
    droll_des, dpitch_des, dyaw_des = dangs_des.x, dangs_des.y, dangs_des.z

    # Moments
    kp, kd = 60, 200  # real robots

   # kp, kd = 220, 400  # Simulation
    #kp, kd = 320, 150
    #kp, kd = 0,0
    #kp, kd = 10, 4

    mx_des = kp * (roll_des - roll2) + kd * (droll_des - droll2)
    #my_des = kp * (pitch_des - pitch2) + kd * (dpitch_des - dpitch2)

    # Thrust for robot in the centroid.
    thrust2_pwm = twist2.linear.z

    # thrust in grams for robot 2
    c1, c2, c3, pwm_max = -0.670900, 0.193200, 13.065200, 60000.
    thrust2 = ((thrust2_pwm / pwm_max - c1) / c2)**2 - c3


    width = .116

    # Total force1: thrust for each robot
    comp = mx_des * (width/(2*width**2 + 3))
    #comp = 0.5*(mx_des/width)


    f1 = thrust2 - comp
    f2 = thrust2
    f3 = thrust2 + comp

    if f1<0: f1 = 0
    if f3<0: f3 = 0


    #if f1<=0 or f3<=0:
    #    print "saturation!", thrust2
    #print (f1, thrust2, comp)


    # PWM thrust
    f2pwm = lambda forc: (c1 + c2 * math.sqrt(c3 + forc)) * pwm_max  # force to pwm
    #f1_pwm, f2_pwm, f3_pwm = f2pwm(f1), f2pwm(f2), f2pwm(f3)
    f1_pwm = f2pwm(f1)
    f2_pwm = f2pwm(f2)
    f3_pwm = f2pwm(f3)

    print f2_pwm, f1_pwm - f2_pwm
    if f1_pwm > pwm_max : f1_pwm = pwm_max
    if f2_pwm > pwm_max : f2_pwm = pwm_max
    if f3_pwm > pwm_max : f3_pwm = pwm_max

    pub1.publish(_mod_twist(twist2, f1_pwm))
    pub2.publish(_mod_twist(twist2, f2_pwm))
    pub3.publish(_mod_twist(twist2, f3_pwm))


def _euler_from_quaterion(quat):
    quaternion = (quat.x, quat.y, quat.z, quat.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    return roll, pitch, yaw


def callbacallback_odom1(odom):
    global odom1
    odom1 = odom

def callbacallback_odom2(odom):
    global odom2
    odom2 = odom

def callbacallback_odom3(odom):
    global odom3
    odom3 = odom

def callback_2(ctwist2):
    #global pub2
    global twist2
    if pub2 is None:
        return
    twist2 = ctwist2
#
# def callback_3(ctwist3):
#     #global pub3
#     global twist3
#     if pub3 is None:
#         return
#     twist3 = ctwist3


def listener():
    global pub1, pub2, pub3
    rospy.init_node('attitud_controller', anonymous=True)

    # Topics toward the robots
    pub1 = rospy.Publisher('/crazy01/cmd_vel', Twist, queue_size=0)
    pub2 = rospy.Publisher('/crazy02/cmd_vel', Twist, queue_size=0)
    pub3 = rospy.Publisher('/crazy03/cmd_vel', Twist, queue_size=0)

    # Topics from robocontroller
    # rospy.Subscriber('/crazy01/cmd_vel2', Twist, callback_1)
    rospy.Subscriber('/crazy02/cmd_vel2', Twist, callback_2)
    # rospy.Subscriber('/crazy03/cmd_vel2', Twist, callback_3)

    # Trajectory

    # Odometry


    rospy.Subscriber('/crazy01/odom', Odometry, callbacallback_odom1)
    rospy.Subscriber('/crazy02/odom', Odometry, callbacallback_odom2)
    rospy.Subscriber('/crazy03/odom', Odometry, callbacallback_odom3)


    rospy.Subscriber('/crazy02/so3_cmd', SO3Command, callbacallback_trajectory)



    rospy.spin()


if __name__ == '__main__':
    listener()
