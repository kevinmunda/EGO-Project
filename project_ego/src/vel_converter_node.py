#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from ego_msgs.msg import EgoTwist2DUnicycle

twist_msg = EgoTwist2DUnicycle()
segway_des_vel_pub = rospy.Publisher('/segway_des_vel', EgoTwist2DUnicycle, queue_size=1)

def convert_vel(msg):
    twist_msg.ForwardVelocity = msg.linear.x
    twist_msg.YawRate = msg.angular.z
    segway_des_vel_pub.publish(twist_msg)
    

def main():
    rospy.init_node("vel_converter_node")

    cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, convert_vel)

    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        r.sleep()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass