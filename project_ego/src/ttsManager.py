import rospy
import pygame
import random

from gtts import gTTS
from io import BytesIO

from pygame import mixer

from project_ego.msg import Event
from project_ego.srv import Reply, ReplyRequest, ReplyResponse

from utils_data import responses, REQ_CORRECT, REQ_INCORRECT, REQ_INCOMPLETE

class ttsManager():

    #############################################################################
    # CONSTRUCTOR
    #############################################################################

    def __init__(self):
        # SERVICE
        self.tts_service = rospy.Service('tts_reply', Reply, self.tts_reply)
        
        # VARIABLES
        self.events = ['greeting_ev', 'event_info_ev']
        
    #############################################################################
    # SERVICE FUNCTIONS
    #############################################################################
    
    def speak(self, reply):
        mp3_fp = BytesIO()
        tts = gTTS(reply, lang='en', tld='com.au')
        tts.write_to_fp(mp3_fp)
        mp3_fp.seek(0)
        mixer.init()
        mixer.music.load(mp3_fp, 'mp3')
        mixer.music.play()
    
    def greeting_reply(self, event_id):
        responses_list = responses[event_id]
        reply = random.choice(responses_list)
        self.speak(reply)
    
    def tts_reply(self, req):
        event_id = req.event_id
        if(event_id in self.events):
            rospy.set_param("/isSpeaking", True)
            if(event_id == "greeting_ev"):
                self.greeting_reply(event_id)
            elif(event_id == "event_info_ev"):
                print("Sono qui")
                pass        
            rospy.sleep(3)
            rospy.set_param("/isSpeaking", False)
            return ReplyResponse(0)
        else:
            print("No reply set for this event")
            return ReplyResponse(1)
        
