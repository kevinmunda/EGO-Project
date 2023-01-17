REQ_CORRECT = "req_correct"
REQ_INCORRECT = "req_incorrect"
REQ_INCOMPLETE = "req_incomplete"


#############################################################################
# TTS REPLIES TO EVENTS
#############################################################################

responses = {
    'greeting_ev': ['hi', 'hello', 'greetings'],
    
    'event_info_ev': {
        'req_correct': {
            'event_found': [" next event is "],
            'no_event_found': ["I'm sorry there are no more events for the date specified"]
        },
        'req_incorrect': ["I'm sorry, i have no info for this date." +
                         "Please, ask again specifying if you want to know about today's or tomorrow's event"],
        'req_incomplete': ["Would you like to know today's or tomorrow's next event?"]
    },
    
    'navigation_ev': {
        'set_goal': {
            'req_correct': ["Follow me, i'll take you there", "Okay, let's go"],
            'req_incorrect': ["I'm sorry, there is no place here with that name." +
                                "Please, choose one of the available locations"]
        },
        'goal_reached': ["Destination reached!"]
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

###############################################################################
# NAVIGATION GOALS
###############################################################################

# Coordinates in the order [pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w]
navigation_goals = {
    'bathroom': [-10.5446844101, 3.93157744408, 0.0, 0.0, 0.0, 0.454964694193, 0.890509476108],
    'wardrobe': [-13.1860618591, 3.8917632103, 0.0, 0.0, 0.0, 0.915059292398, 0.403319341708],
    'lounge': [-9.63719940186, -4.42856502533, 0.0, 0.0, 0.0, 0.888304853122, -0.459254273709]
}

###############################################################################
# FUNCTIONS
###############################################################################

def readEventInfoTxt():
    events_dict = dict.fromkeys(['today', 'tomorrow'])
    events_dict['today'] = {}
    events_dict['tomorrow'] = {}
    f = open("/home/kevin/catkin_ws/src/project_ego/utils/event_info.txt" )
    for line in f:
        line = line.strip('\n')
        event_info = line.split('-')
        events_dict[event_info[0]][event_info[2]] = event_info[1]
    f.close()
    return events_dict

