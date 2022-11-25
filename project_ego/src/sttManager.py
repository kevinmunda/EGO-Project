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
        self.matcher = Matcher(self.nlp.vocab, validate=True)

        # PATTERNS
        pattern = [{"LOWER": {"IN": ["hi", "hello"]}}]
        self.matcher.add("GREETING", [pattern], on_match=self.match_callback)

        # PUBLISHERS
        self.event_catcher_pub = rospy.Publisher('/event_catcher', Event, queue_size=1)
        rospy.sleep(1)
    
    #############################################################################
    # NLP MATCH CALLBACKS
    #############################################################################

    def match_callback(self, matcher, doc, i, matches):
        match_id, start, end = matches[i]
        string_id = self.nlp.vocab.strings[match_id]
        if string_id == "GREETING":
            print("Found a " + string_id + " match")
            msg = Event()
            msg.event_id = string_id.lower()
            self.event_catcher_pub.publish(msg)



    #############################################################################
    # FUNCTIONS
    #############################################################################
    
    def listen(self):
        """
        # Record audio from microphone
        with sr.Microphone() as source:
            print("Say something!")
            audio = self.r.listen(source)
        
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
        """
        
        transcription = "Hi, I'm Kevin"

        # Create a DOC object from the text
        doc = self.nlp(transcription)

        # Find matches with defined patterns
        matches = self.matcher(doc)

