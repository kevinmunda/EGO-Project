#!/usr/bin/env python3

import rospy
from pynput.keyboard import Key, Listener

from ego_msgs.msg import EgoTwist2DUnicycle


twist_msg = EgoTwist2DUnicycle()
segway_des_vel_pub = rospy.Publisher('/segway_des_vel', EgoTwist2DUnicycle, queue_size=1)

def check_move(key):
    print('\nYou Entered {0} \n'.format(key))
    
    if key == Key.up or key == 'w':
        twist_msg.ForwardVelocity = 0.5
        twist_msg.YawRate = 0.0
        segway_des_vel_pub.publish(twist_msg)

    if key == Key.left or key == 'a':
        twist_msg.ForwardVelocity = 0.0
        twist_msg.YawRate = 0.1
        segway_des_vel_pub.publish(twist_msg)
    
    if key == Key.right or key == 'd':
        twist_msg.ForwardVelocity = 0.0
        twist_msg.YawRate = -0.1
        segway_des_vel_pub.publish(twist_msg)
    
    if key == Key.down or key == 's':
        twist_msg.ForwardVelocity = 0.0
        twist_msg.YawRate = 0.0
        segway_des_vel_pub.publish(twist_msg)
        
    if key == Key.delete:
        # Stop listener
        return False

def main():
    rospy.init_node("teleop_node")

    while not rospy.is_shutdown():
        with Listener(on_press=check_move) as listener:
            listener.join()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass