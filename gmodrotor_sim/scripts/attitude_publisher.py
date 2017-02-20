#!/usr/bin/env python



import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

pub = None
robot_name = rospy.get_param('~robot_name', 'crazy01')






def listener():
    global pub
    global robot_name
    rospy.init_node('attitud_controller', anonymous=True)

    pub = rospy.Publisher('/'+robot_name+'/morse_cmd_vel', Float32MultiArray, queue_size=0)

    while not rospy.is_shutdown(): 
        rpyt = Float32MultiArray()
        rpyt.data = [0,0,1.1,.3]
        pub.publish(rpyt)


    rospy.spin()

if __name__ == '__main__':
    listener()









