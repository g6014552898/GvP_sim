#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
import tf2_ros
import geometry_msgs.msg
import numpy as np
import sys
import os
import threading

lock = threading.Lock()

# # goal position on map [px, py, oz, ow]
# goal_pos = [
            # [0.0, 0.0, 0.0, 1.0],        # start point
            # [0.0, 0.0, 0.0, 1.0]        # return to start point
            # ]
def movebase_client(client, g_pos, frame_id):
    client.wait_for_server()
    goal_n = MoveBaseGoal()
    goal_n.target_pose.header.frame_id = frame_id
    goal_n.target_pose.header.stamp = rospy.Time.now()
    goal_n.target_pose.pose.position.x = g_pos[0]
    goal_n.target_pose.pose.position.y = g_pos[1]
    goal_n.target_pose.pose.position.z = 0.0
    goal_n.target_pose.pose.orientation.z = g_pos[2]
    goal_n.target_pose.pose.orientation.w = g_pos[3]

    client.send_goal(goal_n)
    wait = client.wait_for_result(rospy.Duration(15))
    res = client.get_state()
    return res

def check_current_pos(trans):
    # print(trans.transform.translation)
    # print(trans.transform.rotation)
    current_pos = [trans.transform.translation.x, 
                    trans.transform.translation.y, 
                    trans.transform.rotation.z, 
                    trans.transform.rotation.w]
    return current_pos

def clear_costmaps():
    print('Force clear costmaps')
    # os.system("rosservice call /move_base/clear_costmaps")
    rospy.wait_for_service('move_base/clear_costmaps')
    try:
        clear_costmaps = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
def loop_run(client, goal_pos, idx=0, skip=0, frame_id="map"):
    goal_pos_len = len(goal_pos)
    print("set goal %d/%d: %s" % (idx+1, goal_pos_len, goal_pos[idx]))
    res = movebase_client(client, goal_pos[idx], frame_id)
    print("return from movebase_client:%d" %res)
    # print(res)
    
    # PENDING         = 0   # The goal has yet to be processed by the action server
    # ACTIVE          = 1   # The goal is currently being processed by the action server
    
    # PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            # # and has since completed its execution (Terminal State)
    # SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
    # ABORTED         = 4   # The goal was aborted during execution by the action server due
                            # # to some failure (Terminal State)
    # REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            # # because the goal was unattainable or invalid (Terminal State)
    # PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            # # and has not yet completed execution
    # RECALLING       = 7   # The goal received a cancel request before it started executing,
                            # # but the action server has not yet confirmed that the goal is canceled
    # RECALLED        = 8   # The goal received a cancel request before it started executing
                            # # and was successfully cancelled (Terminal State)
    # LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                            # # sent over the wire by an action server
    
    if goal_pos_len > idx+1:
        if res in [1, 2, ]: # repeat current goal
            print('continue..')
            skip += 1
            if skip > 2:
                idx += 1
                print('goal not reachable, skip to next goal')
            # clear_costmaps()
        elif res in [3, ]: # continue next goal
            idx += 1
            skip = 0
            # clear_costmaps()
            print('success, send next goal')
        elif res in [4, 5, ]: # continue next goal1
            skip += 1
            clear_costmaps()
            if skip == 2:
                idx += 1
                print('goal not reachable, skip to next goal')
            if skip > 3: # manual reset
                idx += 1
                os.system("rosservice call /move_base/clear_costmaps")
        else:
            print('return else')
            clear_costmaps()
    else: # end of waypoint
        idx = 0
        clear_costmaps()
        # sys.exit(0)
    return (goal_pos, idx, skip, res)

def main(goal):
    goal_pos = goal
    idx = 0
    skip = 0
    trans = None
    try:
        rospy.init_node('move_command_1')
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rate = rospy.Rate(5.0)
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        while not rospy.is_shutdown():
            try:
                trans = tfBuffer.lookup_transform("map","base_link", rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue
            rate.sleep()
            (goal_pos, idx, skip, res) = loop_run(client, goal_pos, idx, skip)
    except rospy.ROSException as e:
        print(e)

if __name__ == '__main__':
    print("Nothing")