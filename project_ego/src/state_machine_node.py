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
from smach_ros import MonitorState

from project_ego.srv import Reply, ReplyRequest
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ego_msgs.msg import EgoTwist2DUnicycle
from std_msgs.msg import String
from project_ego.msg import Event
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray, GoalID
from sensor_msgs.msg import LaserScan

from utils_data import REQ_CORRECT, REQ_INCORRECT, REQ_INCOMPLETE
from utils_data import event_info_ff, fillEventInfoFF, resetDict
from utils_data import navigation_goals

#################################################################################
#### FUNCTIONS ##################################################################
#################################################################################

def setTTSRequest(event_id, req_type, req_spec=''):
    # TTS reply
    rospy.wait_for_service('tts_reply')
    reply = rospy.ServiceProxy('tts_reply', Reply)
    try:
        req = ReplyRequest(event_id, req_type, req_spec)
        response = reply(req)
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
    return response


#################################################################################
#### STATE CLASSES ##############################################################
#################################################################################

# EVENT HANDLER STATE
class EventHandler(State):
    def __init__(self):
        # List of possible events
        self.events = ['greeting_ev', 'stop_ev', 'event_info_ev', 'navigation_ev']
        self.outcomes = self.events
        self.outcomes.append('unknown_event')
        State.__init__(self, outcomes=self.outcomes, 
                        input_keys=['msg'],
                        output_keys=['msg'])
    
    def execute(self, userdata):
        rospy.loginfo('Executing EVENT HANDLER')
        event_id = userdata.msg.event_id
        if(event_id in self.events):
            return event_id
        else:
            return 'unknown_event'

# EVENT INFO STATES
# State that manages info request's about real events
class EventInfoHandler(State):
    def __init__(self):
        State.__init__(self, outcomes=['interaction_concluded', 'interaction_pending', 
                                        'interaction_suspended', 'aborted'], 
                        input_keys=['msg'],
                        output_keys=['msg'])

    def execute(self, userdata):
        rospy.loginfo('Executing EVENT INFO HANDLER')
        rospy.set_param("/isInteracting", False)
        event_id = userdata.msg.event_id
        match_tokens = userdata.msg.match.match_tokens
        match_pos = userdata.msg.match.match_pos
        match_dep = userdata.msg.match.match_dep

        req_type = ''
        
        # Fill dict with info to process the request
        fillEventInfoFF(event_info_ff, match_tokens, match_dep)
        
        # CASE: date specified - request complete or incorrect
        if(event_info_ff['event_day']):
            # date specified correctly - request complete
            if(event_info_ff['event_day'] in ['today', 'tomorrow']):
                req_type = REQ_CORRECT
                req_spec = event_info_ff['event_day']
            # date specified incorrectly - request incorrect
            else:
                req_type = REQ_INCORRECT
                req_spec = ''
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
        
        resetDict(event_info_ff)
        if(response.response == 1):
            return 'aborted'
        elif(req_type == REQ_CORRECT):
            return 'interaction_concluded'
        elif(req_type == REQ_INCOMPLETE):
            rospy.set_param("/isInteracting", True)
            return 'interaction_pending'
        elif(req_type == REQ_INCORRECT):
            return 'interaction_suspended'

class EventInfoUpdater(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'], 
                        input_keys=['msg', 'user_reply_msg'],
                        output_keys=['msg'])
    
    def execute(self, userdata):
        rospy.loginfo('Executing EVENT INFO UPDATER HANDLER')
        index = userdata.user_reply_msg.match.match_pos.index('NOUN')
        tmp_poss = userdata.user_reply_msg.match.match_tokens[index]
        tmp_case = "'s "
        userdata.msg.match.match_text = tmp_poss + tmp_case + userdata.msg.match.match_text
        userdata.msg.match.match_tokens.insert(0, tmp_poss)
        userdata.msg.match.match_tokens.insert(1, tmp_case)
        userdata.msg.match.match_pos.insert(0, 'NOUN')
        userdata.msg.match.match_pos.insert(1, 'PART')
        userdata.msg.match.match_dep.insert(0, 'poss')
        userdata.msg.match.match_dep.insert(1, 'case')
        return 'succeeded'

# NAVIGATION STATES
class NavigationHandler(State):
    def __init__(self):
        State.__init__(self, outcomes=['goal_set', 'goal_not_set', 'aborted'], 
                        input_keys=['msg'], output_keys=['msg'])
    
    def setGoal(self, goal_coordinates):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        # Set goal coordinates
        goal.target_pose.pose.position.x = goal_coordinates[0]
        goal.target_pose.pose.position.y = goal_coordinates[1]
        goal.target_pose.pose.position.z = goal_coordinates[2]
        goal.target_pose.pose.orientation.x = goal_coordinates[3]
        goal.target_pose.pose.orientation.y = goal_coordinates[4]
        goal.target_pose.pose.orientation.z = goal_coordinates[5]
        goal.target_pose.pose.orientation.w = goal_coordinates[6]
        return goal

    def execute(self, userdata):
        rospy.loginfo('Executing NAVIGATION HANDLER')
        event_id = userdata.msg.event_id
        match_tokens = userdata.msg.match.match_tokens
        match_pos = userdata.msg.match.match_pos
        # GOAL_NAME FOUND
        if(any(token in navigation_goals.keys() for token in match_tokens)):
            # Define the action client
            client = SimpleActionClient('navigation_server', MoveBaseAction)
            client.wait_for_server()
            # Find the goal name given the index (saved with POS-tag NOUN)
            goal_name = match_tokens[match_pos.index('NOUN')]
            # Retrieve goal coordinates given the goal name
            goal_coordinates = navigation_goals[goal_name]
            #goal_coordinates = [1.7, -10.5, 0.0, 0.0, 0.0, 0.0, 1.0]
            # Define the goal for move_base
            goal = self.setGoal(goal_coordinates)
            # Send the goal to the server and wait response
            client.send_goal(goal)
            wait = client.wait_for_result(timeout=rospy.Duration(5))
            if not wait:
                rospy.logerr("Result from server takes too long...")
            else:
                # get_state == SUCCEEDED == 3
                if(client.get_state() == 3):
                    response = setTTSRequest(event_id, REQ_CORRECT, 'set_goal')
                    if(response.response == 0):
                        return 'goal_set'
                    else:
                        return 'aborted'
                # get_state == ABORTED == 4
                elif(client.get_state() == 4):
                    print('SERVER ABORTED - SOME ERROR OCCURED')
                    return 'aborted'
        # GOAL_NAME NOT FOUND
        else:
            response = setTTSRequest(event_id, REQ_INCORRECT, 'set_goal')
            return 'goal_not_set'  

class NavigationServer:
    def __init__(self, name):
        self.server = SimpleActionServer(name, MoveBaseAction, 
                                        execute_cb=self.execute_cb)
        self.server.start()
    
    def execute_cb(self, goal):
        rospy.loginfo('Executing NAVIGATION_SERVER')
        pose_msg = PoseStamped()
        pose_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        rospy.sleep(1)
        pose_msg.header.frame_id = goal.target_pose.header.frame_id
        pose_msg.header.stamp = goal.target_pose.header.stamp
        pose_msg.pose.position.x = goal.target_pose.pose.position.x
        pose_msg.pose.position.y = goal.target_pose.pose.position.y
        pose_msg.pose.position.z = goal.target_pose.pose.position.z
        pose_msg.pose.orientation.x = goal.target_pose.pose.orientation.x
        pose_msg.pose.orientation.y = goal.target_pose.pose.orientation.y
        pose_msg.pose.orientation.z = goal.target_pose.pose.orientation.z
        pose_msg.pose.orientation.w = goal.target_pose.pose.orientation.w
        result = pose_pub.publish(pose_msg)
        if result == None:
            self.server.set_succeeded()
        else:
            self.server.set_aborted()

class NavigationMonitor(State):
    def __init__(self):
        # FLAGS
        self.first_msg = True # This flag is used to avoid considering "goal reached" messages coming from previous navigation
        self.goal_reached = False # Becomes true when the goal is reached
        self.navigation_stopped = False # Becomes true when the navigation is stopped
        self.obstacle_detected = False # Becomes true when an obstacle is detected at a specific distance
        
        # PUBLISHERS
        self.move_base_cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.segway_des_vel_pub = rospy.Publisher('/segway_des_vel', EgoTwist2DUnicycle, queue_size=1)
        rospy.sleep(1)
        
        State.__init__(self, outcomes=['goal_reached', 'navigation_stopped'], 
                        input_keys=['msg'], output_keys=[])
    
    def checkGoalStatus(self, msg):
        if(len(msg.status_list) > 0):
            if(self.first_msg and msg.status_list[0].status == 1):
                print("Goal accepted")
                self.first_msg = False
            if(not self.first_msg and msg.status_list[0].status == 3):
                print("Goal reached")
                self.goal_reached = True
            else:
                self.goal_reached = False
        else:
            return
    
    def checkStop(self, msg):
        if(msg.event_id == 'stop_ev'):
            self.navigation_stopped = True
            empty_goal = GoalID()
            self.move_base_cancel_pub.publish(empty_goal)
        else:
            return
    
    def checkLaser(self, msg):
        if(any(scan > 0.0 and scan < 1.0 for scan in msg.ranges)):
            print('Obstacle detected')
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def execute(self, userdata):
        # FLAGS
        self.first_msg = True
        self.goal_reached = False
        self.navigation_stopped = False
        
        # SUBSCRIBERS
        self.move_base_result_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.checkGoalStatus, queue_size=1)
        self.event_trigger_sub = rospy.Subscriber('/event_trigger', Event, self.checkStop, queue_size=1)
        self.laser_scan_sub = rospy.Subscriber('/scan', LaserScan, self.checkLaser, queue_size=1)
        
        while(not self.goal_reached and not self.navigation_stopped):
            if(self.obstacle_detected):
                vel_msg = EgoTwist2DUnicycle()
                vel_msg.ForwardVelocity = 0.0
                vel_msg.YawRate = 0.0
                self.segway_des_vel_pub.publish(vel_msg)
            
        self.move_base_result_sub.unregister()
        self.event_trigger_sub.unregister()
        if(self.navigation_stopped):
            return 'navigation_stopped'
        elif(self.goal_reached):
            event_id = userdata.msg.event_id
            response = setTTSRequest(event_id, '', 'goal_reached')
            return 'goal_reached'


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
    if result == None and response.response == 0:
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

def user_data_cb(userdata, msg):
    userdata.user_reply_msg = msg
    return False

#################################################################################
#### MAIN #######################################################################
#################################################################################        

def main():
    rospy.init_node('state_machine_node')

    sm = StateMachine(outcomes=['succeeded', 'aborted'])
    navigationServer = NavigationServer('navigation_server')
    
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
                                    'navigation_ev':'NAVIGATION_HANDLER_STATE',
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
        
        # EVENT-INFO STATES ########################################################
        StateMachine.add('EVENT_INFO_MONITOR_STATE', 
                        MonitorState("/user_reply", Event, user_data_cb,
                                    input_keys=['msg'],
                                    output_keys=['msg', 'user_reply_msg']),
                        transitions={'valid':'EVENT_MONITOR_STATE', 
                                    'invalid':'EVENT_INFO_UPDATER_STATE',
                                    'preempted':'EVENT_MONITOR_STATE'},
                        remapping={'msg':'msg'})
        
        StateMachine.add('EVENT_INFO_UPDATER_STATE', EventInfoUpdater(),
                        transitions={'succeeded':'EVENT_INFO_HANDLER_STATE'},
                        remapping={'msg':'msg', 'user_reply_msg':'user_reply_msg'})
        
        StateMachine.add('EVENT_INFO_HANDLER_STATE', EventInfoHandler(),
                        transitions={'interaction_concluded':'EVENT_MONITOR_STATE',
                                    'interaction_pending':'EVENT_INFO_MONITOR_STATE',
                                    'interaction_suspended':'EVENT_MONITOR_STATE',
                                    'aborted':'aborted'},
                        remapping={'msg':'msg'})
        
        # NAVIGATION STATES ########################################################
        StateMachine.add('NAVIGATION_HANDLER_STATE', NavigationHandler(),
                        transitions={'goal_set':'NAVIGATION_MONITOR_STATE',
                                    'goal_not_set':'EVENT_MONITOR_STATE',
                                    'aborted':'aborted'},
                        remapping={'msg':'msg'})
        
        StateMachine.add('NAVIGATION_MONITOR_STATE', NavigationMonitor(),
                        transitions={'goal_reached':'EVENT_MONITOR_STATE',
                                    'navigation_stopped':'EVENT_MONITOR_STATE'},
                        remapping={'msg':'msg'})
        
    sis = smach_ros.IntrospectionServer('server', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()