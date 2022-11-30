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

#################################################################################
#### STATE CLASSES ##############################################################
#################################################################################

class EventHandler(State):
    def __init__(self):
        # List of possible events
        self.events = ['greeting_ev', 'stop_ev']
        self.outcomes = self.events
        self.outcomes.append('unknown_event')
        State.__init__(self, outcomes=self.outcomes, input_keys=['event_id'])
    
    def execute(self, userdata):
        rospy.loginfo('Executing EVENT HANDLER')
        if(userdata.event_id in self.events):
            return userdata.event_id
        else:
            return 'unknown_event'

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

    sm = StateMachine(outcomes=['succeeded', 'aborted'], 
                    input_keys=[])
    
    with sm:
        # EVENT MONITOR & HANDLER STATES ########################################
        StateMachine.add('EVENT_MONITOR_STATE', 
                        MonitorState("/event_trigger", Event, event_cb,
                                    output_keys=['event_id']),
                        transitions={'valid':'EVENT_MONITOR_STATE', 
                                    'invalid':'EVENT_HANDLER_STATE',
                                    'preempted':'EVENT_MONITOR_STATE'})
        
        StateMachine.add('EVENT_HANDLER_STATE',
                        EventHandler(),
                        transitions={'greeting_ev':'GREETING_EVENT_STATE',
                                    'stop_ev':'STOP_EVENT_STATE',
                                    'unknown_event':'EVENT_MONITOR_STATE'},
                        remapping={'event_id':'event_id'})
        
        # EVENT CALLBACKS STATES ################################################
        StateMachine.add('GREETING_EVENT_STATE', CBState(greeting_cb),
                        transitions={'succeeded':'EVENT_MONITOR_STATE',
                                    'aborted':'aborted'})
        
        StateMachine.add('STOP_EVENT_STATE', CBState(stop_cb),
                        transitions={'succeeded':'EVENT_MONITOR_STATE',
                                    'aborted':'aborted'})
        
    sis = smach_ros.IntrospectionServer('server', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()