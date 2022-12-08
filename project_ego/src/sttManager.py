import rospy

import speech_recognition as sr
import spacy
from spacy.matcher import Matcher, DependencyMatcher

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
        self.token_matcher = Matcher(self.nlp.vocab, validate=True)
        self.dependency_matcher = DependencyMatcher(self.nlp.vocab, validate=True)

        # PATTERNS
        pattern = [{"LOWER": {"IN": ["hi", "hello"]}}]
        self.token_matcher.add("GREETING", [pattern], on_match=self.match_cb)
        
        pattern = [{"LOWER": "stop"}]
        self.token_matcher.add("STOP", [pattern], on_match=self.match_cb)
        
        """
        pattern = [
            {
                "RIGHT_ID": "anchor",
                "RIGHT_ATTRS": {"POS": "AUX", "ORTH": "is"}
            },
            {
                "LEFT_ID": "anchor",
                "REL_OP": ">",
                "RIGHT_ID": "anchor_adverb",
                "RIGHT_ATTRS": {"DEP": "advmod", "LOWER": "when"}
            },
            {
                "LEFT_ID": "anchor",
                "REL_OP": ">",
                "RIGHT_ID": "anchor_object",
                "RIGHT_ATTRS": {"DEP": "nsubj", "LOWER": "event"}
            },
            {
                "LEFT_ID": "anchor_object",
                "REL_OP": ">",
                "RIGHT_ID": "anchor_object_modifier",
                "RIGHT_ATTRS": {"DEP": "amod", "LOWER": "next"}
            }
        ]

        self.dependency_matcher.add("EVENT_INFO", [pattern])
        """
        
        pattern = [
            {"DEP": "poss", "LOWER": {"IN": ["today", "tomorrow"]}, "OP": "*"},
            {"DEP": "case", "OP": "*"},
            {"DEP": "amod", "LOWER": "next"},
            {"DEP": "nsubj", "LOWER": "event"}
        ]

        self.token_matcher.add("EVENT_INFO", [pattern], on_match=self.match_cb)

        # PUBLISHERS
        self.event_catcher_pub = rospy.Publisher('/event_catcher', Event, queue_size=1)
        rospy.sleep(1)
    
    #############################################################################
    # CALLBACKS
    #############################################################################

    def match_cb(self, matcher, doc, i, matches):
        match_id, start, end = matches[i]
        string_id = self.nlp.vocab.strings[match_id]
        print("Found a " + string_id + " match")
        msg = Event()
        msg.event_id = string_id.lower() + "_ev"
        span = doc[start:end]
        msg.match.match_text = span.text
        for token in span:
            msg.match.match_tokens.append(token.text)
            msg.match.match_dep.append(token.dep_)
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
        

        #transcription = "When is today's next event?"
        
        # Create a DOC object from the text
        doc = self.nlp(transcription)
        
        # Find matches w.r.t defined patterns
        matches = self.token_matcher(doc)


