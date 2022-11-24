#!/usr/bin/env python3

import rospy
from sttManager import sttManager

def main():
    rospy.init_node('stt_node')

    stt = sttManager()

    rate = rospy.Rate(30)

    #while not rospy.is_shutdown():
    
    stt.listen()
    rospy.spin()

    #rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass