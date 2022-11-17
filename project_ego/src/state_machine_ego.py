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
from smach_ros import SimpleActionState

from project_ego.msg import moveEgoAction, moveEgoGoal

from ego_msgs.msg import EgoTwist2DUnicycle
from std_msgs.msg import String


#################################################################################
#### STATE CLASSES ##############################################################
#################################################################################

class Idle(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing IDLE')
        return 'succeeded'

#################################################################################
#### STATE CALLBACKS ############################################################
#################################################################################

@smach.cb_interface(input_keys=['fv_cb_in', 'yr_cb_in'], 
                    output_keys=['fv_cb_out', 'yr_cb_out'], 
                    outcomes=['succeeded', 'aborted'])
def move_ego_cb(userdata):
    rospy.loginfo('Executing MOVE_EGO_CB')
    twist_msg = EgoTwist2DUnicycle()
    segway_des_vel_pub = rospy.Publisher('/segway_des_vel', 
                                        EgoTwist2DUnicycle, queue_size=1)
    rospy.sleep(1)
    twist_msg.ForwardVelocity = userdata.fv_cb_in
    twist_msg.YawRate = userdata.yr_cb_in
    result = segway_des_vel_pub.publish(twist_msg)
    rospy.sleep(5)
    userdata.fv_cb_out = 0.0
    userdata.yr_cb_out = 0.0
    if result == None:
        return 'succeeded'
    else:
        return 'aborted'

@smach.cb_interface(input_keys=['fv_cb_in', 'yr_cb_in'], 
                    outcomes=['succeeded', 'aborted'])
def stop_ego_cb(userdata):
    rospy.loginfo('Executing STOP_EGO_CB')
    twist_msg = EgoTwist2DUnicycle()
    segway_des_vel_pub = rospy.Publisher('/segway_des_vel', 
                                        EgoTwist2DUnicycle, queue_size=1)
    rospy.sleep(1)
    twist_msg.ForwardVelocity = userdata.fv_cb_in
    twist_msg.YawRate = userdata.yr_cb_in
    result = segway_des_vel_pub.publish(twist_msg)
    if result == None:
        return 'succeeded'
    else:
        return 'aborted'

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
#### GOAL CALLBACKS #############################################################
#################################################################################

def move_ego_goal_cb(userdata, goal):
    move_ego_goal = moveEgoGoal()
    move_ego_goal.forwardVelocity = userdata.forwardVelocity_goal
    move_ego_goal.yawRate = userdata.yawRate_goal
    move_ego_goal.executionTime = userdata.executionTime_goal
    return move_ego_goal

#################################################################################
#### ACTION SERVERS #############################################################
#################################################################################

class MoveEgoServer:
    def __init__(self, name):
        self._sas = SimpleActionServer(name, moveEgoAction,
                                        execute_cb=self.execute_cb)
    
    def execute_cb(self, goal):
        rospy.loginfo('Executing MOVE_EGO_SERVER')
        twist_msg = EgoTwist2DUnicycle()
        segway_des_vel_pub = rospy.Publisher('/segway_des_vel', 
                                        EgoTwist2DUnicycle, queue_size=1)
        rospy.sleep(1)
        twist_msg.ForwardVelocity = goal.forwardVelocity
        twist_msg.YawRate = goal.yawRate
        result = segway_des_vel_pub.publish(twist_msg)
        rospy.sleep(goal.executionTime)
        if result == None:
            self._sas.set_succeeded()
        else:
            self._sas.set_aborted()


#################################################################################
#### MAIN #######################################################################
#################################################################################        

def main():
    rospy.init_node('state_machine_ego')

    moveEgoServer = MoveEgoServer('move_ego_server')

    # Create the state machine
    sm = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'], 
                    input_keys=[])
    sm.userdata.sm_forwardVelocity = 0.5
    sm.userdata.sm_yawRate = 0.0
    sm.userdata.sm_executionTime = 5
    
    with sm:
        # Add the states
        StateMachine.add('IDLE', Idle(),
                        transitions={'succeeded':'MOVE_EGO'})
        
        """
        StateMachine.add('MOVE_EGO', CBState(move_ego_cb),
                        transitions={'succeeded':'STOP_EGO',
                                    'aborted':'aborted'},
                        remapping={'fv_cb_in':'sm_forwardVelocity',
                                    'yr_cb_in':'sm_yawRate',
                                    'fv_cb_out':'sm_forwardVelocity',
                                    'yr_cb_out':'sm_yawRate'})
        
        StateMachine.add('STOP_EGO', CBState(stop_ego_cb),
                        transitions={'succeeded':'GREETING',
                                    'aborted':'aborted'},
                        remapping={'fv_cb_in':'sm_forwardVelocity',
                                    'yr_cb_in':'sm_yawRate'})
        
        """

        StateMachine.add('MOVE_EGO', 
                        SimpleActionState('move_ego_server', moveEgoAction,
                                        goal_cb=move_ego_goal_cb,
                                        input_keys=['forwardVelocity_goal',
                                                    'yawRate_goal',
                                                    'executionTime_goal']),
                        transitions={'succeeded':'STOP_EGO',
                                    'aborted':'aborted'},
                        remapping={'forwardVelocity_goal':'sm_forwardVelocity',
                                    'yawRate_goal':'sm_yawRate',
                                    'executionTime_goal':'sm_executionTime'})

        StateMachine.add('STOP_EGO', 
                        SimpleActionState('move_ego_server', moveEgoAction,
                                        goal=moveEgoGoal(0.0, 0.0, 1)),
                        transitions={'succeeded':'GREETING',
                                    'aborted':'aborted'})

        StateMachine.add('GREETING', CBState(greeting_cb),
                        transitions={'succeeded':'succeeded',
                                    'aborted':'aborted'})
        
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()