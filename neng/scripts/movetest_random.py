#!/usr/bin/env python

import movetest
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
import tf2_ros
import geometry_msgs.msg
import numpy as np
import sys
import os

def random_pos(goal):

    goal[0] += np.random.randint(0, 500)/500.0
    goal[1] += np.random.randint(-500, 500)/500.0
    goal[2] += np.random.randint(-100, 100)/100.0
    goal[3] += np.random.randint(-100, 100)/100.0
    return goal

def main():
    idx = 0
    skip = 0
    res = 0
    trans = None
    
    try:
        rospy.init_node('move_random_1')
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
            # alway use idx = 0 for single target
            (goal_pos, idx, skip, res) = movetest.loop_run(client, 
                                                    [random_pos([0.0, 0.0, 0.0, 1.0])],
                                                    0, 
                                                    skip,
                                                    "base_link"
                                                    )
            
    except rospy.ROSException as e:
        print(e)
    
if __name__ == '__main__':
    main()