#!/usr/bin/env python3

import rospy
from sttManager import sttManager

def main():
    rospy.init_node('stt_node')

    rospy.set_param("/isSpeaking", False)

    stt = sttManager()

    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        if(rospy.get_param("/isSpeaking") == False):
            stt.listen()
        else:
            print("speaking")
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass