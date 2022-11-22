#!/usr/bin/env python3

import rospy

import speech_recognition as sr
import spacy

def main():
    rospy.init_node('stt_node')

    #############################################################################
    # RECOGNIZER INSTANCE & SETTINGS
    #############################################################################
    r = sr.Recognizer()
    r.energy_threshold = 200
    # This should be set to True in situations where ambient noise level is unpredictable
    r.dynamic_energy_threshold = False
    language = "en-US"
    #language = "it-IT"

    
    #############################################################################
    # NLP MODEL
    #############################################################################
    nlp = spacy.load("en_core_web_sm")

    # Rate object
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        # Record audio from microphone
        with sr.Microphone() as source:
            print("Say something!")
            audio = r.listen(source)
        
        # Transcript the audio
        try:
            transcription = r.recognize_google(audio, language=language)
            print("You said: " + transcription)
        except sr.UnknownValueError:
            print("Unable to understand the audio")
        except sr.RequestError as e:
            print("Some error occured; {0}".format(e))

        doc = nlp(transcription)
        for token in doc:
            print(token.text, token.pos_, token.dep_)

        rate.sleep()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass