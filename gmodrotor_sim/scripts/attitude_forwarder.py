#!/usr/bin/env python



import rospy
import math 
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


pub = None


def callback(twist):
    global pub
    if pub is None:
        return
    
    # Roll pitch yaw trust
    rpyt = Float32MultiArray()
    #
    c1 = -0.6709
    c2 = 0.1932
    c3 = 13.0652
    thrust = ((twist.linear.z / 60000. - c1) / c2)**2 - c3  # Force in grams
    thrust = 9.81 * thrust / 1000.  # Force in Newtons
    
    roll = math.radians(twist.linear.y)
    pitch = -math.radians(twist.linear.x)
    yaw = 6 * math.radians(twist.angular.z)
    rpyt.data = [roll, pitch, yaw, thrust]
    pub.publish(rpyt)


def listener():
    global pub
    rospy.init_node('attitud_controller', anonymous=True)

    rospy.Subscriber('cmd_vel', Twist, callback)
    pub = rospy.Publisher('morse_cmd_vel', Float32MultiArray, queue_size=0)
    rospy.spin()


if __name__ == '__main__':
    listener()









