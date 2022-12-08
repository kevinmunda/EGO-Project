import rospy

from project_ego.msg import Event

class eventManager():
    
    #############################################################################
    # CONSTRUCTOR
    #############################################################################
    
    def __init__(self):
        
        # PUBLISHERS
        self.event_trigger_pub = rospy.Publisher("/event_trigger", Event, queue_size=1)
        rospy.sleep(1)
        
        # SUBSCRIBERS
        self.event_catcher_sub = rospy.Subscriber("/event_catcher", Event, self.eventTrigger)
    
    #############################################################################
    # CALLBACKS FUNCTIONS
    #############################################################################

    def eventTrigger(self, msg):
        self.event_trigger_pub.publish(msg)
