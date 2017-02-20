#!/usr/bin/env python



import rospy
import tf
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped


# Publisher
pub = None
# Velocity
vel = None
# Robot name
robot_name = rospy.get_param('~robot_name', 'crazy01')



def callback_pose(pose):
    global pub
    if pub is None:
        return
    if vel is None:
        return


    # Roll pitch yaw trust
    odom = Odometry()
    odom.pose.pose.position.x = pose.pose.position.x
    odom.pose.pose.position.y = pose.pose.position.y
    odom.pose.pose.position.z = pose.pose.position.z

    odom.pose.pose.orientation.x = pose.pose.orientation.x
    odom.pose.pose.orientation.y = pose.pose.orientation.y
    odom.pose.pose.orientation.z = pose.pose.orientation.z
    odom.pose.pose.orientation.w = pose.pose.orientation.w
     
    # Velocities
    odom.twist.twist.linear.x = vel.twist.linear.x
    odom.twist.twist.linear.y = vel.twist.linear.y
    odom.twist.twist.linear.z = vel.twist.linear.z
    

    odom.twist.twist.angular.x = vel.twist.angular.x
    odom.twist.twist.angular.y = vel.twist.angular.y
    odom.twist.twist.angular.z = vel.twist.angular.z


    pub.publish(odom)



def callback_vel(velocity):
    global vel
    vel = velocity
    

def listener():
    global pub
    global robot_name
    rospy.init_node('odometry_morse', anonymous=True)
    
    # Odometry publisher
    pub = rospy.Publisher('odom', Odometry, queue_size=0)

    # Subscribe to pose and vel from Morse topics
    rospy.Subscriber('pose', PoseStamped, callback_pose)
    rospy.Subscriber('vel', TwistStamped, callback_vel)
    

        
    
    rospy.spin()

if __name__ == '__main__':
    listener()









