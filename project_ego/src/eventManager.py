import rospy

from project_ego.msg import Event

class eventManager():
    
    #############################################################################
    # CONSTRUCTOR
    #############################################################################
    
    def __init__(self):
        
        # VARIABLES
        self.events = ['greeting_ev']

        # PUBLISHERS
        self.event_trigger_pub = rospy.Publisher("/event_trigger", Event, queue_size=1)
        rospy.sleep(1)
        
        # SUBSCRIBERS
        self.event_catcher_sub = rospy.Subscriber("/event_catcher", Event, self.eventTrigger)
    
    #############################################################################
    # CALLBACKS FUNCTIONS
    #############################################################################

    def eventTrigger(self, msg):
        if msg.event_id == "greeting":
            tmp_msg = Event()
            tmp_msg.event_id = 'ciao'
            self.event_trigger_pub.publish(tmp_msg)

