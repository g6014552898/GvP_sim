#!/usr/bin/env python

import movetest
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
import tf
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg
import nav_msgs.srv
import numpy as np
import sys
import threading

class Thread(threading.Thread):
    def __init__(self, t, *args):
        threading.Thread.__init__(self, target=t, args=args)
        self.start()

base_link_pose_lock = threading.Lock()
grid_map_lock = threading.Lock()
target_point_lock = threading.Lock()

plan_lock = threading.Lock()

generated_points = None

base_link_pose = geometry_msgs.msg.PoseStamped()
grid_map = nav_msgs.msg.OccupancyGrid()

def set_base_link_pose(pose):
    global base_link_pose
    with base_link_pose_lock:
        base_link_pose = pose
def get_base_link_pose():
    tmp = None
    with base_link_pose_lock:
        tmp = base_link_pose
    return tmp

def set_target_point(points):
    global generated_points
    with target_point_lock:
        generated_points = points
def get_target_point():
    tmp = None
    with target_point_lock:
        tmp = generated_points
    return tmp

def set_grid_map(map):
    global grid_map
    with grid_map_lock:
        grid_map = map
def get_grid_map():
    tmp = None
    with grid_map_lock:
        tmp = grid_map
    return tmp
    
    
def occupancygrid_to_numpy(msg):
    data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    return np.ma.array(data, mask=data==-1, fill_value=-1)

def get_clicked(pose):
    # print [pose.point.x, pose.point.y]
    x = pose.point.x
    y = pose.point.y
    # check_cost([x, y])
    circle_points = gen_circle_points(rc, [x, y], yawc)
    target_points = []
    if len(circle_points) > 0:
        for _point, ang in circle_points:
            target_points.append(_point)
        set_target_point(target_points)
    else:
        print "No valid point"
        set_target_point(None)

def point_on_circle(center, radius, angle, yaw):
    orientation = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(yaw))
    x = center[0] + (radius * np.cos(np.deg2rad(angle)))
    y = center[1] + (radius * np.sin(np.deg2rad(angle)))
    qz = orientation[2]
    qw = orientation[3]
    return x,y,qz,qw
    
def get_current_xyzw():
    _pose = get_base_link_pose().pose
    curr_point = [_pose.position.x, 
                _pose.position.y,
                _pose.orientation.z,
                _pose.orientation.w]
    return curr_point

def gen_circle_points(r=2.5, center=[0,0], yaw=0, r_min=1.0, r_step=0.1, ang_step=10, cell_cost=5):
    set_target_point(None)
    print "generate position on circle..."
    targets = []
    valid_targets = []
    map = get_grid_map()
    map_data = occupancygrid_to_numpy(map)
    for ang in range(0, 360, int(ang_step)):
        points_tmp = []
        # default 10cm step
        end_r = int((r+r_step)*10)
        step_r = int(r_step*10)
        for r_s in range(0, end_r, step_r):
            x, y, qz, qw = point_on_circle([center[0], center[1]],
                                        r_s/10.0,
                                        ang,
                                        yaw+ang)
            points_tmp.append([x, y, qz, qw, r_s/10.0])
    # check for cost value on point; repack data with cost value
        points_with_cost = check_cost(points_tmp, map, map_data)
        targets.append([points_with_cost, ang])
    curr_point = get_current_xyzw()
    # can't check plan while moving
    with plan_lock:
        client.wait_for_server()
        client.cancel_all_goals()
        client.wait_for_result()
        global idx
        
    # check for reachable and visible target at each angle
        for points_with_cost, ang in targets: 
            last_los_point = None
    # check for LoS to circle center
            center_cost = points_with_cost[0][5]
            for point in points_with_cost:
                if point[5] <= cell_cost: # get only point that cost < cell_cost
                    if point[4] >= r_min: # ignore inside r_min area
                        if last_los_point is None:
                            last_los_point = point
                            break
                            # get only one point that nearest click spot

    # check last_los_point is valid position to move on
            if last_los_point is not None:
    # check reachability from move_base and get plan
                # poses = get_plan(curr_point, last_los_point)
                # poses is list of pose from make_plan service
                # if len(poses) > 0:
                # filter for only reachable point
                    # valid_targets.append([last_los_point, poses, ang])                
                valid_targets.append([last_los_point, ang])                
                    # update curr_point to selected
                curr_point = last_los_point
    # release lock and re-index
        idx = 0

    return valid_targets
    
def get_plan(current, target):
    # make_plan_topic = "move_base/GlobalPlanner/make_plan"
    make_plan_topic = "move_base/GlobalPlanner/make_plan"
    rospy.wait_for_service(make_plan_topic)
    make_plan = rospy.ServiceProxy(make_plan_topic, nav_msgs.srv.GetPlan)
    
    start = geometry_msgs.msg.PoseStamped()
    start.header.frame_id = "map"
    start.pose.position.x = current[0]
    start.pose.position.y = current[1]
    start.pose.orientation.z = current[2]
    start.pose.orientation.w = current[3]

    goal = geometry_msgs.msg.PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = target[0]
    goal.pose.position.y = target[1]
    goal.pose.orientation.z = target[2]
    goal.pose.orientation.w = target[3]
    
    tolerance = 0.0
    plan_response = make_plan(start, goal, tolerance)
    # return list contains point in path
    return plan_response.plan.poses

def tf_to_pose(_tf):
    tf_tr = _tf.transform.translation
    tf_rt = _tf.transform.rotation

    _pose = geometry_msgs.msg.PoseStamped()
    _pose.header = _tf.header
    _pose.pose.position = tf_tr
    _pose.pose.orientation = tf_rt
    
    return _pose
    
def costmap_listen(msg):
    set_grid_map(msg)
    
def check_cost(points, map=None, map_data=None): # swap x, y in map array
    if map_data is None or map is None:
        map = get_grid_map()
        map_data = occupancygrid_to_numpy(map)
    result_list = []
    for point in points:
        # print point

        grid_x = np.uint32((point[0] - map.info.origin.position.y) / map.info.resolution) #input_y
        grid_y = np.uint32((point[1] - map.info.origin.position.x) / map.info.resolution) #input_x
        point.append(map_data[grid_y, grid_x])
        result_list.append(point)
    
    return result_list
    
def run_tf_listen():
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(5.0)
    try:
        while not rospy.is_shutdown():
            try:
                _tf = tfBuffer.lookup_transform("map","base_link", rospy.Time())
                
                _pose = tf_to_pose(_tf)
                
                set_base_link_pose(_pose)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
            rate.sleep()
    except Exception as e:
        print(e)

def main():
    global client
    global idx
    idx = 0
    skip = 0
    res = 0
    pose_target = None
    try:
        rospy.init_node('move_circle_1')
        tf_listner = Thread(run_tf_listen)
        rospy.Subscriber("clicked_point", geometry_msgs.msg.PointStamped, get_clicked)
        rospy.Subscriber("/move_base/global_costmap/costmap", nav_msgs.msg.OccupancyGrid, costmap_listen)
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            pose_target = get_target_point()
            # use idx = 0 for single target
            if pose_target is not None:
                # print get_base_link_pose()
                last_idx = idx
                with plan_lock:
                    (goal_pos, idx, skip, res) = movetest.loop_run(client,
                                                    pose_target,
                                                    idx, 
                                                    skip,
                                                    "map"
                                                    )
                    # end of waypoint when last_idx == size-1
                    if last_idx == len(pose_target)-1: 
                        print "Done"
                        set_target_point(None)
                        print "Waiting for next point..."
            rate.sleep()
            
            
    except Exception as e:
        print(e)
    
if __name__ == '__main__':
    print "default param [r] [yaw]: 1 90"
    global rc
    global yawc
    rc = 1
    yawc = 90
    if len(sys.argv) > 1:
        if len(sys.argv) >= 2:
            rc = float(sys.argv[1])
        if len(sys.argv) >= 3:
            yawc = float(sys.argv[2])
    main()