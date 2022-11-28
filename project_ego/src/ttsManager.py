import rospy

from project_ego.msg import Event
from project_ego.srv import Reply, ReplyRequest, ReplyResponse

class ttsManager():

    #############################################################################
    # CONSTRUCTOR
    #############################################################################

    def __init__(self):
        # SERVICE
        self.tts_service = rospy.Service('tts_reply', Reply, self.reply)

        # VARIABLES
        self.events = ['greeting_ev']
        
        # SUBSCRIBERS
        self.event_trigger_sub = rospy.Subscriber("/event_trigger", Event, self.event_cb)

    #############################################################################
    # CALLBACKS
    #############################################################################
    
    def event_cb(self, msg):
        rospy.wait_for_service('tts_reply')
        reply = rospy.ServiceProxy('tts_reply', Reply)
        try:
            response = reply(msg.event_id)
            if(response == 0):
                print("Reply executed correctly")
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    #############################################################################
    # SERVICE FUNCTIONS
    #############################################################################
    
    def reply(self, req):
        if(req.event_id in self.events):
            print(req.event_id)
            return ReplyResponse(0)
        else:
            print("No reply set for this event")
            return ReplyResponse(1)
        
