#!/usr/bin/env python3

import rospy
from eventManager import eventManager

def main():
    rospy.init_node('event_handler_node')

    em = eventManager()

    rospy.spin()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass