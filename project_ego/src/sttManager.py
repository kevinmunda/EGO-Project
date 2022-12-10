import rospy

import speech_recognition as sr
import spacy
from spacy.matcher import Matcher

from project_ego.msg import Event

class sttManager():
    
    #############################################################################
    # CONSTRUCTOR
    #############################################################################
    
    def __init__(self):
        # RECOGNIZER INSTANCE & SETTINGS
        self.r = sr.Recognizer()
        self.r.energy_threshold = 200
        self.r.dynamic_energy_threshold = False  # This should be set to True when ambient noise level is unpredictable
        self.language = "en-US" # alternatives: "it-IT"

        # NLP MODEL
        self.nlp = spacy.load("en_core_web_sm")
        self.event_matcher = Matcher(self.nlp.vocab, validate=True)
        self.user_reply_matcher = Matcher(self.nlp.vocab, validate=True)
        
        # EVENT PATTERNS
        pattern = [{"LOWER": {"IN": ["hi", "hello"]}}]
        self.event_matcher.add("GREETING", [pattern])
        
        pattern = [{"LOWER": "stop"}]
        self.event_matcher.add("STOP", [pattern])
        
        pattern = [
            {"DEP": "poss", "OP": "*"},
            {"DEP": "case", "OP": "*"},
            {"DEP": "amod", "LOWER": "next"},
            {"DEP": "nsubj", "LOWER": "event"}
        ]
        self.event_matcher.add("EVENT_INFO", [pattern])

        # USER-REPLY EVENT-INFO PATTERNS
        pattern = [{"LOWER": {"IN": ["today", "tomorrow"]}}]
        self.user_reply_matcher.add("EVENT_INFO_CORRECT_REPLY", [pattern])

        pattern = [{"LOWER": {"NOT_IN": ["today", "tomorrow"]}}]
        self.user_reply_matcher.add("EVENT_INFO_WRONG_REPLY", [pattern])

        # PUBLISHERS
        self.event_catcher_pub = rospy.Publisher('/event_catcher', Event, queue_size=1)
        self.user_reply_pub = rospy.Publisher('/user_reply', Event, queue_size=1)
        rospy.sleep(1)
    
    #############################################################################
    # CALLBACKS
    #############################################################################
    
    def match_cb(self, doc, matches):
        match_id, start, end = matches[:1][0]
        string_id = self.nlp.vocab.strings[match_id]
        print("Found a " + string_id + " match")
        msg = Event()
        msg.event_id = string_id.lower() + "_ev"
        span = doc[start:end]
        msg.match.match_text = span.text
        for token in span:
            msg.match.match_tokens.append(token.text)
            msg.match.match_pos.append(token.pos_)
            msg.match.match_dep.append(token.dep_)
        if(rospy.get_param("/isInteracting") == True):
            self.user_reply_pub.publish(msg)
        else:
            self.event_catcher_pub.publish(msg)
    
    #############################################################################
    # FUNCTIONS
    #############################################################################
    
    def listen(self):
        # Record audio from microphone
        with sr.Microphone() as source:
            print("Say something!")
            audio = self.r.listen(source)
        
        if(rospy.get_param("/isSpeaking") == True):
            return

        # Transcript the audio
        try:
            transcription = self.r.recognize_google(audio, language=self.language)
            print("You said: " + transcription)
        except sr.UnknownValueError:
            print("Unable to understand the audio")
            transcription = ''
        except sr.RequestError as e:
            print("Some error occured; {0}".format(e))
            transcription = ''
        
        # Create a DOC object from the text
        doc = self.nlp(transcription)
        
        if(rospy.get_param('/isInteracting') == True):
            matches = self.user_reply_matcher(doc)
        else:
            # Find matches w.r.t defined patterns
            matches = self.event_matcher(doc)
        
        if(len(matches) > 0):
            self.match_cb(doc, matches)




