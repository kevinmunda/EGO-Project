import rospy
import pygame
import random

from datetime import datetime

from gtts import gTTS
from io import BytesIO

from pygame import mixer

from project_ego.msg import Event
from project_ego.srv import Reply, ReplyRequest, ReplyResponse

from utils_data import responses, REQ_CORRECT, REQ_INCORRECT, REQ_INCOMPLETE
from utils_data import readEventInfoTxt

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
    
    def event_info_reply(self, event_id, req_type, req_spec):
        responses_list = responses[event_id][req_type]
        if(req_type == REQ_CORRECT):
            # in this case responses_list is a dict
            events_dict = readEventInfoTxt()
            events_day_dict = events_dict[req_spec]
            event_found = False
            
            now = datetime.now()
            old_dt = now.replace(hour=23, minute=59, second=0, microsecond=0)
            for key in events_day_dict.keys():
                current_dt_split = key.split(':')
                current_dt = now.replace(hour=int(current_dt_split[0]), minute=int(current_dt_split[1]), second=0, microsecond=0)
                if(req_spec == 'today'):
                    if(now < current_dt and  current_dt < old_dt):
                        event_found = True
                        event_name = events_day_dict[key]
                        event_hour = key
                        old_dt = current_dt
                    else:
                        pass
                else:
                    if(current_dt < old_dt):
                        event_found = True
                        event_name = events_day_dict[key]
                        event_hour = key
                        old_dt = current_dt
                    else:
                        pass
            if(event_found):
                reply = random.choice(responses_list['event_found'])
                reply = req_spec + "'s " + reply + event_name + " at " + event_hour
            else:
                reply = random.choice(responses_list['no_event_found'])    
            self.speak(reply)
        else:
            reply = random.choice(responses_list)
            self.speak(reply)

    def tts_reply(self, req):
        event_id = req.event_id
        req_type = req.req_type
        req_spec = req.req_spec

        if(event_id in self.events):
            rospy.set_param("/isSpeaking", True)
            if(event_id == "greeting_ev"):
                self.greeting_reply(event_id)
            elif(event_id == "event_info_ev"):
                self.event_info_reply(event_id, req_type, req_spec)
            rospy.sleep(3)
            rospy.set_param("/isSpeaking", False)
            return ReplyResponse(0)
        else:
            print("No reply set for this event")
            return ReplyResponse(1)
        
