REQ_CORRECT = "req_correct"
REQ_INCORRECT = "req_incorrect"
REQ_INCOMPLETE = "req_incomplete"


#############################################################################
# TTS REPLIES TO EVENTS
#############################################################################
responses = {
    'greeting_ev': ['hi', 'hello', 'greetings'],
    'event_info_ev': {
        'req_correct': [" next event is "],
        'req_incorrect': ["I'm sorry, i have no info for this date." +
                         "Please ask again specifying if you want to know about today's or tomorrow's event"],
        'req_incomplete': ["Would you like to know today's or tomorrow's next event?"]
    }
}

###############################################################################
# FORM-FILLING DICTS
###############################################################################
event_info_ff = dict.fromkeys(['event_day'], None)


###############################################################################
# FORM-FILLING FUNCTIONS
###############################################################################
def fillEventInfoFF(dict, tokens_list, dep_list):
    if(any(elem == 'poss' for elem in dep_list)):
        dict['event_day'] = tokens_list[dep_list.index('poss')]

def resetDict(dict):
    for key in dict.keys():
        dict[key] = None


