#!/usr/bin/env python
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
import math

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, PoseStamped, PointStamped
from std_msgs.msg import Header, ColorRGBA

waypoint_list = [
            [-2.0, 2.0, 1.0, 0.01], 
            [2.0, 2.0, -0.7, 0.7], 
            [2.0, -2.0, 0.0, 1.0],
            [-2.0, -2.0, 0.7, 0.7]
            ]

class Thread(threading.Thread):
  def __init__(self, t, *args):
    threading.Thread.__init__(self, target=t, args=args)
    self.start()
    
base_link_pose_lock = threading.Lock()
cost_map_lock = threading.Lock()
static_map_lock = threading.Lock()
target_point_lock = threading.Lock()
plan_lock = threading.Lock()

run_scan = False
generated_points = None
base_link_pose = PoseStamped()
cost_map = nav_msgs.msg.OccupancyGrid()
static_map = nav_msgs.msg.OccupancyGrid()

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

def set_static_map(map):
  global static_map
  with static_map_lock:
    static_map = map

def get_static_map():
  tmp = None
  with static_map_lock:
    tmp = static_map
  return tmp

def set_cost_map(map):
  global cost_map
  with cost_map_lock:
    cost_map = map

def get_cost_map():
  tmp = None
  with cost_map_lock:
    tmp = cost_map
  return tmp

def occupancygrid_to_numpy(msg):
  data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.width, msg.info.height)
  return np.ma.array(data, fill_value=-1)

def ang_diff(source, target):
  a = source[0] - target[0]
  b = source[1] - target[1]
  r = calculate_distance(source[:2], target[:2])
  if r == 0:
    return 0, 0, 0
    
  theta_a = math.acos(a/r)
  theta_b = math.asin(b/r)
  
  return theta_a, theta_b, r
  
def points_on_line(source, target, r_step=0.05, r_fix=0):
  point_list = []
  # find theta to target on map frame
  theta_a, theta_b, r = ang_diff(source, target)
  
  if r == 0:
    if r_fix == 0:
      r = math.sqrt(2)
    else:
      r = r_fix
  
  if r_fix == 0:
    # find x, y for every r (m) step from target to source
    r_step = int(r_step*100)
    r_int = int(r*100)+r_step
    for r in range (0, r_int, r_step):
      r_cal = r*0.01
      x = target[0] + (r_cal*math.cos(theta_a))
      y = target[1] + (r_cal*math.sin(theta_b))
      point_list.append([x, y])
  else:
    x = target[0] + (r_fix*math.cos(theta_a))
    y = target[1] + (r_fix*math.sin(theta_b))
    point_list.append([x, y])
  # output list position from target to source
  return point_list
  

def point_on_circle(center, radius, angle, yaw):
  orientation = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(yaw))
  x = center[0] + (radius * np.cos(np.deg2rad(angle)))
  y = center[1] + (radius * np.sin(np.deg2rad(angle)))
  qz = orientation[2]
  qw = orientation[3]
  return x,y,qz,qw
  
def get_clicked(pose):
  # print [pose.point.x, pose.point.y]
  
  x = pose.point.x
  y = pose.point.y

  points = check_cost_diff([[x, y]], debug=1)
  print points

def calculate_distance(a,b):
  return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

def get_current_xyzw():
  _pose = get_base_link_pose().pose
  curr_point = [_pose.position.x, 
              _pose.position.y,
              _pose.orientation.z,
              _pose.orientation.w]
  return curr_point
  
def get_plan(current, target):
  # make_plan_topic = "move_base/GlobalPlanner/make_plan"
  make_plan_topic = "move_base/GlobalPlanner/make_plan"
  rospy.wait_for_service(make_plan_topic)
  make_plan = rospy.ServiceProxy(make_plan_topic, nav_msgs.srv.GetPlan)
  start = PoseStamped()
  start.header.frame_id = "map"
  start.pose.position.x = current[0]
  start.pose.position.y = current[1]
  start.pose.orientation.z = current[2]
  start.pose.orientation.w = current[3]
  goal = PoseStamped()
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
  _pose = PoseStamped()
  _pose.header = _tf.header
  _pose.pose.position = tf_tr
  _pose.pose.orientation = tf_rt
  return _pose
  
def costmap_listener(msg):
  set_cost_map(msg)
  
def check_cost_diff(points, cost_map=None, cost_data=None, map=None, map_data=None, debug=0):
  if cost_map is None:
    cost_map = get_cost_map()
  if cost_data is None:
    cost_data = occupancygrid_to_numpy(cost_map)
  if map is None:
    map = get_static_map()
  if map_data is None:
    map_data = occupancygrid_to_numpy(map)
    
  result_list = []
  for point in points:
    # print point
    map_res = map.info.resolution
    grid_x_static_map = np.uint32((point[0] - map.info.origin.position.x) / map_res)
    grid_y_static_map = np.uint32((point[1] - map.info.origin.position.y) / map_res)
    
    grid_x_cost_map = np.uint32((point[0] - cost_map.info.origin.position.x) / cost_map.info.resolution)
    grid_y_cost_map = np.uint32((point[1] - cost_map.info.origin.position.y) / cost_map.info.resolution)
    map_cost = 100
    point_cost = 0
    try:
      try:
        map_cost = 0
        map_cost += map_data[grid_y_static_map+2, grid_x_static_map+2]
        map_cost += map_data[grid_y_static_map+2, grid_x_static_map-2]
        map_cost += map_data[grid_y_static_map-2, grid_x_static_map-2]
        map_cost += map_data[grid_y_static_map-2, grid_x_static_map+2]
        map_cost += map_data[grid_y_static_map+2, grid_x_static_map]
        map_cost += map_data[grid_y_static_map-2, grid_x_static_map]
        map_cost += map_data[grid_y_static_map, grid_x_static_map+2]
        map_cost += map_data[grid_y_static_map, grid_x_static_map-2]
        map_cost += map_data[grid_y_static_map+1, grid_x_static_map+1]
        map_cost += map_data[grid_y_static_map+1, grid_x_static_map-1]
        map_cost += map_data[grid_y_static_map-1, grid_x_static_map-1]
        map_cost += map_data[grid_y_static_map-1, grid_x_static_map+1]
        map_cost += map_data[grid_y_static_map+1, grid_x_static_map]
        map_cost += map_data[grid_y_static_map-1, grid_x_static_map]
        map_cost += map_data[grid_y_static_map, grid_x_static_map+1]
        map_cost += map_data[grid_y_static_map, grid_x_static_map-1]
        map_cost += map_data[grid_y_static_map, grid_x_static_map]
        map_cost /= 18.0
      except:
        if debug:
          print ("Input point is out of area.")

      try:
        point_cost = 0
        point_cost += cost_data[grid_y_cost_map+1, grid_x_cost_map+1]
        point_cost += cost_data[grid_y_cost_map+1, grid_x_cost_map-1]
        point_cost += cost_data[grid_y_cost_map-1, grid_x_cost_map-1]
        point_cost += cost_data[grid_y_cost_map-1, grid_x_cost_map+1]
        point_cost += cost_data[grid_y_cost_map+1, grid_x_cost_map]
        point_cost += cost_data[grid_y_cost_map-1, grid_x_cost_map]
        point_cost += cost_data[grid_y_cost_map, grid_x_cost_map+1]
        point_cost += cost_data[grid_y_cost_map, grid_x_cost_map-1]
        point_cost += cost_data[grid_y_cost_map, grid_x_cost_map]
        point_cost /= 9.0
      except:
        if debug:
          print ("Input point is out of area.")
      cost_diff = (abs(point_cost) - abs(map_cost))
      result_list.append([cost_diff, point_cost, map_cost, point])
    except IndexError:
      if debug:
        print ("Input point is out of area.")
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
    
def movebase_client(client, g_pos, frame_id, debug=0):
  if debug:
    print "wait_for_server..."
  client.wait_for_server()
  goal_n = MoveBaseGoal()
  goal_n.target_pose.header.frame_id = frame_id
  goal_n.target_pose.header.stamp = rospy.Time.now()
  goal_n.target_pose.pose.position.x = g_pos[0]
  goal_n.target_pose.pose.position.y = g_pos[1]
  goal_n.target_pose.pose.position.z = 0.0
  goal_n.target_pose.pose.orientation.z = g_pos[2]
  goal_n.target_pose.pose.orientation.w = g_pos[3]
  if debug:
    print "sending goal..."
    print goal_n.target_pose.pose
  client.send_goal(goal_n)
  wait = client.wait_for_result(rospy.Duration(30))
  res = client.get_state()
  return res

def move_goal(client, goal_pos, frame_id="map", debug=0):
  res = movebase_client(client, goal_pos, frame_id)
  if debug:
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
  return res

# def get_r_in_target_list(target):
  # return target[0][4]

# def get_theha_in_target_list(target):
  # return target[1]
  
  # x = (x*math.cos(theta)) - (y*math.sin(theta))
  # y = (x*math.sin(theta)) + (y*math.cos(theta))

def transformations(pose, ang=0, cen=[0, 0], trans=[0, 0], scale=[1, 1]):
  theta = math.radians(ang)
  R = np.array([
   [math.cos(theta), -math.sin(theta), 0], 
   [math.sin(theta), math.cos(theta), 0],
   [0, 0, 1]
   ])
  
  T = np.array([
  [1, 0, trans[0]],
  [0, 1, trans[1]],
  [0, 0, 1]
  ])
  
  S = np.array([
  [scale[0], 0, 0],
  [0, scale[1], 0],
  [0, 0, 1]
  ])

  RST = np.dot(np.dot(R, S), T)

  Pt = np.array([[pose[0] - cen[0]], 
                [pose[1] - cen[1]], 
                [1]])
                
  Ct = np.dot(RST, Pt)

  x = cen[0] + Ct[0][0]
  y = cen[1] + Ct[1][0]
  
  return x, y

def find_next_point(last, ref, min_area, r_scan):
  next_point_list = []
  last_ref_point = points_on_line(last, ref, r_fix=r_scan)[-1]
  for ang in range(0,120):
    ref_point = transformations(last_ref_point, ang, cen=ref)
    # find object surface position
    next_point = get_nearest_object_point(ref_point, ref, 99)

    if next_point is not None:
      distA_B = calculate_distance(last, next_point)
      # distRef_B = calculate_distance(ref, next_point)
      next_point_list.append([next_point, ang, distA_B])
      
      if distA_B >= min_area and len(next_point_list) > 1:
        break
  return next_point_list

def find_next_pose(last, ref_obj, min_area, min_r, debug=0, marker_publisher=None):
  # find next target 
  next_point_list = find_next_point(last, ref_obj,min_area, min_r*2)

  if len(next_point_list) > 1:
    #
    #     c
    #    /|\
    #   a-d-b
    #    \|/
    #     c
    #     |
    #     e
    
    a = last
    b = next_point_list[-1][0]
    
    a_b = points_on_line(a, b, 0.01)
    
    d = a_b[len(a_b)/2]
    
    c = transformations(b, 90, cen=d)

    r_pose = min_r + calculate_distance(c, d)
    
    e = points_on_line(d, c, r_fix=r_pose)[-1]

    try:
      cost_e = check_cost_diff([e])[0]
    except IndexError:
      cost_e = -1

    if cost_e[1] != 0 or cost_e[2] != 0:
      for r in range(10):
        r_pose += 0.1
        e = points_on_line(d, c, r_fix=r_pose)[-1]
        cost_e = check_cost_diff([e])[0]
        if cost_e[1] == 0 and cost_e[2] == 0:
          break
        else:
          return None, last
    
    head_ref = transformations(e, 
                    trans=[-c[0],-c[1]])
    
    x, y = round(e[0], 2), round(e[1], 2)
    
    theta = math.atan2(head_ref[1], head_ref[0])

    if theta < 0:
      yaw_pose = math.pi + theta
    else:
      yaw_pose = theta - math.pi
    
    # camera heading adjust
    
    # yaw_pose -= math.pi/2

    orientation = tf.transformations.quaternion_from_euler(0, 0, yaw_pose)
    
    _pose = [x, y, orientation[2], orientation[3]]
    if debug:
      print "==========find_next_pose=========="
      print "a:", a
      print "b:", b
      print "c:", c
      print "d:", d
      print "pose:", _pose
      print "heading:", math.degrees(yaw_pose)
      print "cost e:", cost_e
    
    if marker_publisher is not None:
      show_text_in_rviz(marker_publisher, 
                [b[0], b[1], 0, 1],
                "B", 
                3, 
                0,
                [1.0, 0.1, 0.1, 1.0])
      show_text_in_rviz(marker_publisher, 
                [d[0], d[1], 0, 1],
                "D", 
                4, 
                0,
                [1.0, 0.1, 0.1, 1.0])
      show_text_in_rviz(marker_publisher, 
                [c[0], c[1], 0, 1],
                "C", 
                5, 
                0,
                [1.0, 0.1, 0.1, 1.0])
    
    return _pose, b
  else:
    return None, last
  
def get_nearest_object_point(source, ref, cost=99):
  points = points_on_line(source, ref)
  map = get_static_map()
  map_data = occupancygrid_to_numpy(map)
  cost_map = get_cost_map()
  cost_data = occupancygrid_to_numpy(cost_map)
  points_with_cost = check_cost_diff(points, cost_map, cost_data, map, map_data)
  object_point = None
  for cost_diff, cost_point_val, map_cost_val, point in points_with_cost:
    if cost_point_val >= cost and map_cost_val == 0:
      object_point = point
      # break
  return object_point

def get_nearest_object_pose(object_point, ref_obj, min_r):
  if object_point is not None:
    r = calculate_distance(object_point[:2], ref_obj)
    r_pose = min_r + r

    points = points_on_line(object_point, ref_obj, r_fix=r_pose)
    if len(points) > 0:
      x, y = round(points[0][0], 2), round(points[0][1], 2)
      
      theta = math.atan2(y, x)
    
      if theta < 0:
        yaw_pose = math.pi + theta
      else:
        yaw_pose = theta - math.pi

      orientation = tf.transformations.quaternion_from_euler(0, 0, yaw_pose)
      
      _pose = [x, y, orientation[2], orientation[3]]
      return _pose
    else:
      print "len(points) = 0"
  return None

def show_text_in_rviz(marker_publisher, p, text, id, time, rgba):
  marker = Marker(
    type=Marker.TEXT_VIEW_FACING,
    id=id,
    lifetime=rospy.Duration(time),
    pose=Pose(Point(p[0], p[1], 0.2), Quaternion(0, 0, p[2], p[3])),
    scale=Vector3(0.1, 0.1, 0.1),
    header=Header(frame_id='map'),
    color=ColorRGBA(rgba[0], rgba[1], rgba[2], rgba[3]),
    text=text)
  marker_publisher.publish(marker)
      
      
def control_main():
  client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
  
  pose_target = None
  object_point = None
  r_pose = None
  rate = rospy.Rate(1.0)
  ref_obj = []
  ref_point = [0.0, 0.0]
  res = 0
  min_r = 1.2 # minimun_distance
  finding = True
  next_waypoint = 0
  
  while not rospy.is_shutdown():
    try:
        if get_current_xyzw()[0] != 0:
          # if len(ref_obj) > 0:
            # finding = False
            # if pose_target is None:
              # object_point = ref_obj

              # ref_point = object_point[:2]
              
              # pose_target, ref_point = find_next_pose(ref_point, 
                            # ref_obj, 
                            # 0.2, 
                            # min_r,
                            # 1,
                            # marker_publisher)
          # else:
          # finding = True
          next_waypoint %= len(waypoint_list)
          pose_target = waypoint_list[next_waypoint]
          next_waypoint += 1

          if pose_target is not None:
            show_text_in_rviz(marker_publisher, pose_target, 
              "Next", 
              9, 
              10,
              [0.2, 0.2, 0.8, 1.0])
            res = move_goal(client, pose_target)
            if res == 3:
              show_text_in_rviz(marker_publisher, pose_target, 
              "Done!!", 
              0, 
              2,
              [0.2, 0.8, 0.2, 1.0])
                
            # if ref_point is not None:
              # show_text_in_rviz(marker_publisher, 
                # [ref_point[0], ref_point[1], 0, 1],
                # "A", 
                # 2, 
                # 0,
                # [1.0, 0.1, 0.1, 1.0])
              # pose_target, ref_point = find_next_pose(ref_point, 
                            # ref_obj, 
                            # 0.2, 
                            # min_r,
                            # 1,
                            # marker_publisher)
            # else:
              # print pose_target, ref_point
              # finding = True
          
          # if finding:
            # obj_list = scan_obj_circle([
            # [1.5, 4], [1.6, 4], [1.7, 4],[1.8, 4], [1.9, 4], [2.0, 4]
            # ], marker_publisher) #marker_publisher
            # if len(obj_list) > 0:
              # ref_obj = obj_list[0]
        rospy.sleep(1)
    except KeyboardInterrupt:
      break

def call_static_map_service():
  try:
    static_map_resp = rospy.ServiceProxy('/static_map', nav_msgs.srv.GetMap)()
    static_map = static_map_resp.map
    set_static_map(static_map)
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

def gen_scan_points(r=2.5, center=[0,0], ang_step=10):
  # print "generate position on circle..."
  point_circle = []
  for ang in range(0, 360, int(ang_step)):
    x, y, qz, qw = point_on_circle([center[0], center[1]],
                                    r,
                                    ang,
                                    ang)
    point_circle.append([[x, y], [qz, qw], ang])
  return point_circle
  
def scan_obj_circle(layers=[[2, 4]], marker=None):
  obj_list = []
  for r , ang in layers:
    x, y, z, w = get_current_xyzw()
    points_scan = gen_scan_points(r,[x, y] , ang)
    ids = r*1000
    for [x, y], [qz, qw], ang in points_scan:
      if marker is not None:
        show_text_in_rviz(marker, 
                  [x, y, qz, qw],
                  "+", 
                  ids, 
                  3,
                  [1.0, 0.1, 0.1, 1.0])
        ids +=1
      for point in check_cost_diff([[x, y]]):
        if point[1] >= 99 and point[2] == 0:
          obj_list.append([x, y])
          print "scan_res",point
          return obj_list  
  return obj_list  
def test():
  print("Start test function")
  rate = rospy.Rate(1.0)
  marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
  while not rospy.is_shutdown():
    try:
      rospy.sleep(1)
    except KeyboardInterrupt:
      break

def main():
  try:
    rospy.init_node('move_scan_1')
    tf_listner = Thread(run_tf_listen)
    rospy.Subscriber("clicked_point", PointStamped, get_clicked)
    rospy.Subscriber("/move_base/local_costmap/costmap", nav_msgs.msg.OccupancyGrid, costmap_listener)
    call_static_map_service()

    control_main()
    #test()
    
  finally:
    tf_listner._Thread__stop()

if __name__ == '__main__':
  print "param [r] [yaw]: 1 0"
  global rc
  global yawc
  rc = 2
  yawc = 0
  if len(sys.argv) > 1:
    if len(sys.argv) >= 2:
      rc = float(sys.argv[1])
    if len(sys.argv) >= 3:
      yawc = float(sys.argv[2])
      
  main()
  
