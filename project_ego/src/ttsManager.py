import rospy
import pygame
import random

from gtts import gTTS
from io import BytesIO

from pygame import mixer

from project_ego.msg import Event
from project_ego.srv import Reply, ReplyRequest, ReplyResponse

from utils_data import *

class ttsManager():

    #############################################################################
    # CONSTRUCTOR
    #############################################################################

    def __init__(self):
        # SERVICE
        self.tts_service = rospy.Service('tts_reply', Reply, self.tts_reply)
        
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
            print(response)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    #############################################################################
    # SERVICE FUNCTIONS
    #############################################################################
    
    def speak(self, event_id):
        responses_list = responses[event_id]
        reply = random.choice(responses_list)
        mp3_fp = BytesIO()
        tts = gTTS(reply, lang='en', tld='com.au')
        tts.write_to_fp(mp3_fp)
        mp3_fp.seek(0)
        mixer.init()
        mixer.music.load(mp3_fp, 'mp3')
        mixer.music.play()

    
    def tts_reply(self, req):
        if(req.event_id in self.events):
            rospy.set_param("/isSpeaking", True)
            self.speak(req.event_id)
            rospy.sleep(3)
            rospy.set_param("/isSpeaking", False)
            return ReplyResponse(0)
        else:
            print("No reply set for this event")
            return ReplyResponse(1)
        
