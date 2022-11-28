#!/usr/bin/env python3

import rospy

from ttsManager import ttsManager

def main():
    rospy.init_node("tts_node")

    tts = ttsManager()
    
    while not rospy.is_shutdown():
        tts.tts_service.spin()

    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
