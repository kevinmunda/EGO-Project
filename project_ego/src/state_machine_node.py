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

from ego_msgs.msg import EgoTwist2DUnicycle
from std_msgs.msg import String
from project_ego.msg import Event

# List of possible events
events = ['greeting_ev']

#################################################################################
#### STATE CLASSES ##############################################################
#################################################################################

class Idle(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing IDLE')
        return 'succeeded'

class EventHandler(State):
    def __init__(self):
        State.__init__(self, outcomes=events, input_keys=['event_id'])
    
    def execute(self, userdata):
        rospy.loginfo('Executing EVENT HANDLER')
        #print(userdata.event_id)
        return userdata.event_id

#################################################################################
#### STATE CALLBACKS ############################################################
#################################################################################

@smach.cb_interface(outcomes=['succeeded', 'aborted'])
def greeting_cb(userdata):
    rospy.loginfo('Executing GREETING_CB')
    string_msg = String()
    gesture_command_pub = rospy.Publisher('/gesture_command', 
                                        String, queue_size=1)
    rospy.sleep(1)
    string_msg.data = 'greeting'
    result = gesture_command_pub.publish(string_msg)
    if result == None:
        return 'succeeded'
    else:
        return 'aborted'

#################################################################################
#### MONITOR STATES CALLBACKS ###################################################
#################################################################################

def event_ch_cb(userdata, msg):
    userdata.event_id = msg.event_id
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

    # Create the state machine
    sm = StateMachine(outcomes=['succeeded', 'aborted'], 
                    input_keys=[])
    
    with sm:
        # Add the states
        StateMachine.add('EVENT_MONITOR_STATE', 
                        MonitorState("/event_trigger", Event, event_ch_cb,
                                    output_keys=['event_id']),
                        transitions={'valid':'EVENT_MONITOR_STATE', 
                                    'invalid':'EVENT_HANDLER_STATE',
                                    'preempted':'EVENT_MONITOR_STATE'})
        
        StateMachine.add('EVENT_HANDLER_STATE',
                        EventHandler(),
                        transitions={'greeting_ev':'GREETING_STATE'},
                        remapping={'event_id':'event_id'})
        
        StateMachine.add('TEST_STATE', Idle(),
                        transitions={'succeeded':'GREETING_STATE'})
        
        StateMachine.add('GREETING_STATE', CBState(greeting_cb),
                        transitions={'succeeded':'EVENT_MONITOR_STATE',
                                    'aborted':'aborted'})
        
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()