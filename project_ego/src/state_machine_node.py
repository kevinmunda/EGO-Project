#!/usr/bin/env python

import roslib
roslib.load_manifest('smach')
roslib.load_manifest('smach_ros')
import rospy
import smach
import smach_ros

from actionlib import *
from actionlib_msgs import *

from smach import StateMachine, State, CBState
from smach_ros import SimpleActionState, MonitorState

from project_ego.msg import moveEgoAction, moveEgoGoal
from project_ego.srv import Reply, ReplyRequest, ReplyResponse

from ego_msgs.msg import EgoTwist2DUnicycle
from std_msgs.msg import String
from project_ego.msg import Event

from utils_data import REQ_CORRECT, REQ_INCORRECT, REQ_INCOMPLETE

#################################################################################
#### STATE CLASSES ##############################################################
#################################################################################

class EventHandler(State):
    def __init__(self):
        # List of possible events
        self.events = ['greeting_ev', 'stop_ev', 'event_info_ev']
        self.outcomes = self.events
        self.outcomes.append('unknown_event')
        State.__init__(self, outcomes=self.outcomes, 
                        input_keys=['msg'],
                        output_keys=['msg'])
    
    def execute(self, userdata):
        event_id = userdata.msg.event_id
        rospy.loginfo('Executing EVENT HANDLER')
        if(event_id in self.events):
            return event_id
        else:
            return 'unknown_event'

# State that manages info request's about real events
class EventInfoHandler(State):
    def __init__(self):
        State.__init__(self, outcomes=['interaction_concluded', 'interaction_pending', 
                                        'interaction_suspended', 'aborted'], 
                        input_keys=['msg'])

    def execute(self, userdata):
        event_id = userdata.msg.event_id
        match_tokens = userdata.msg.match.match_tokens
        match_dep = userdata.msg.match.match_dep

        req_type = ''

        # CASE: date specified - request complete or incorrect
        if(any(elem == 'poss' for elem in match_dep)):
            index = match_dep.index('poss')
            token = match_tokens[index]
            # date specified correctly - request complete
            if(token in ['today', 'tomorrow']):
                req_type = REQ_CORRECT
                req_spec = token
            # date specified incorrectly - request incorrect
            else:
                req_type = REQ_INCORRECT
                req_spec = ''
                pass
        # CASE: date not specified - request incomplete
        else:
            req_type = REQ_INCOMPLETE
            req_spec = ''

        # TTS reply
        rospy.wait_for_service('tts_reply')
        reply = rospy.ServiceProxy('tts_reply', Reply)
        try:
            req = ReplyRequest(event_id, req_type, req_spec)
            response = reply(req)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        if(response == 1):
            return 'aborted'
        elif(req_type == REQ_CORRECT):
            return 'interaction_concluded'
        elif(req_type == REQ_INCOMPLETE):
            return 'interaction_pending'
        elif(req_type == REQ_INCORRECT):
            return 'interaction_suspended'

#################################################################################
#### CALLBACK STATES ############################################################
#################################################################################

@smach.cb_interface(outcomes=['succeeded', 'aborted'], input_keys=['msg'])
def greeting_cb(userdata):
    event_id = userdata.msg.event_id
    rospy.loginfo('Executing GREETING_CB')
    # Publish msg to execute gesture
    string_msg = String()
    gesture_command_pub = rospy.Publisher('/gesture_command', 
                                        String, queue_size=1)
    rospy.sleep(1)
    string_msg.data = 'greeting'
    result = gesture_command_pub.publish(string_msg)
    # TTS reply
    rospy.wait_for_service('tts_reply')
    reply = rospy.ServiceProxy('tts_reply', Reply)
    try:
        req = ReplyRequest(event_id, "", "")
        response = reply(req)
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
    if result == None and response == 0:
        return 'succeeded'
    else:
        return 'aborted'

@smach.cb_interface(outcomes=['succeeded', 'aborted'])
def stop_cb(userdata):
    rospy.loginfo('Executing STOP_CB')
    string_msg = String()
    twist_msg = EgoTwist2DUnicycle()
    gesture_command_pub = rospy.Publisher('/gesture_command', 
                                        String, queue_size=1)
    segway_des_vel_pub = rospy.Publisher('/segway_des_vel',
                                        EgoTwist2DUnicycle, queue_size=1)
    rospy.sleep(1)
    string_msg.data = 'stop'
    twist_msg.ForwardVelocity = 0.0
    twist_msg.YawRate = 0.0
    result = gesture_command_pub.publish(string_msg)
    if result == None:
        pass
    else:
        return 'aborted'
    result = segway_des_vel_pub.publish(twist_msg)
    if result == None:
        return 'succeeded'
    else:
        return 'aborted'

#################################################################################
#### MONITOR STATES CALLBACKS ###################################################
#################################################################################

def event_cb(userdata, msg):
    userdata.msg = msg
    return False


#################################################################################
#### GOAL CALLBACKS #############################################################
#################################################################################

#################################################################################
#### ACTION SERVERS #############################################################
#################################################################################

#################################################################################
#### MAIN #######################################################################
#################################################################################        

def main():
    rospy.init_node('state_machine_node')

    sm = StateMachine(outcomes=['succeeded', 'aborted'], 
                    input_keys=[])
    
    with sm:
        # EVENT MONITOR & HANDLER STATES ########################################
        StateMachine.add('EVENT_MONITOR_STATE', 
                        MonitorState("/event_trigger", Event, event_cb,
                                    output_keys=['msg']),
                        transitions={'valid':'EVENT_MONITOR_STATE', 
                                    'invalid':'EVENT_HANDLER_STATE',
                                    'preempted':'EVENT_MONITOR_STATE'})
        
        StateMachine.add('EVENT_HANDLER_STATE',
                        EventHandler(),
                        transitions={'greeting_ev':'GREETING_EVENT_STATE',
                                    'stop_ev':'STOP_EVENT_STATE',
                                    'event_info_ev':'EVENT_INFO_HANDLER_STATE',
                                    'unknown_event':'EVENT_MONITOR_STATE'},
                        remapping={'msg':'msg'})
        
        # EVENT CALLBACKS STATES ################################################
        StateMachine.add('GREETING_EVENT_STATE', CBState(greeting_cb),
                        transitions={'succeeded':'EVENT_MONITOR_STATE',
                                    'aborted':'aborted'},
                        remapping={'msg':'msg'})
        
        StateMachine.add('STOP_EVENT_STATE', CBState(stop_cb),
                        transitions={'succeeded':'EVENT_MONITOR_STATE',
                                    'aborted':'aborted'})
        
        # CLASSES STATES ########################################################
        StateMachine.add('EVENT_INFO_MONITOR_STATE', 
                        MonitorState("/event_trigger", Event, event_cb,
                                    output_keys=['msg']),
                        transitions={'valid':'EVENT_MONITOR_STATE', 
                                    'invalid':'EVENT_HANDLER_STATE',
                                    'preempted':'EVENT_MONITOR_STATE'})

        StateMachine.add('EVENT_INFO_HANDLER_STATE', EventInfoHandler(),
                        transitions={'interaction_concluded':'EVENT_MONITOR_STATE',
                                    'interaction_pending':'EVENT_INFO_MONITOR_STATE'},
                        remapping={'msg':'msg'})
        
    sis = smach_ros.IntrospectionServer('server', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()