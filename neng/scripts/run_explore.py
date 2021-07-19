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
import cv2
import imutils

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, PoseStamped, PointStamped
from std_msgs.msg import Header, ColorRGBA

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

def occupancyGridToNumpyArray(msg):
  data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width,)
  return np.ma.array(data, fill_value=-1)

def calcAngDiff(source, target):
  a = source[0] - target[0]
  b = source[1] - target[1]
  r = calculateDistance(source[:2], target[:2])
  if r == 0:
    return 0, 0, 0
    
  theta_a = math.acos(a/r)
  theta_b = math.asin(b/r)
  
  return theta_a, theta_b, r

def getClicked(pose):
  # print [pose.point.x, pose.point.y]
  
  x = pose.point.x
  y = pose.point.y

  points = checkCostmapAtPoints([[x, y]], debug=1)
  print points

def calculateDistance(a,b):
  return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

def getCurrentXYZW():
  _pose = get_base_link_pose().pose
  curr_point = [_pose.position.x, 
              _pose.position.y,
              _pose.orientation.z,
              _pose.orientation.w]
  return curr_point
  
def getPlan(current, target):
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

def tfToPose(_tf):
  tf_tr = _tf.transform.translation
  tf_rt = _tf.transform.rotation
  _pose = PoseStamped()
  _pose.header = _tf.header
  _pose.pose.position = tf_tr
  _pose.pose.orientation = tf_rt
  return _pose
  
def costmapTopicListenner(msg):
  set_cost_map(msg)
  
def mapTopicListenner(msg):
  set_static_map(msg)
  processMapData()
  
def checkCostmapAtPoints(points, cost_map=None, cost_data=None, map=None, map_data=None, debug=0):
  if cost_map is None:
    cost_map = get_cost_map()
  if cost_data is None:
    cost_data = occupancyGridToNumpyArray(cost_map)
  if map is None:
    map = get_static_map()
  if map_data is None:
    map_data = occupancyGridToNumpyArray(map)
    
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

def tfListenner():
  tfBuffer = tf2_ros.Buffer()
  listener = tf2_ros.TransformListener(tfBuffer)
  rate = rospy.Rate(5.0)
  try:
    while not rospy.is_shutdown():
      try:
        _tf = tfBuffer.lookup_transform("map","base_link", rospy.Time())
        _pose = tfToPose(_tf)
        set_base_link_pose(_pose)
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        continue
      rate.sleep()
  except Exception as e:
    print(e)
    
def movebaseClient(client, g_pos, frame_id, debug=0):
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

def moveToGoal(client, goal_pos, frame_id="map", debug=0):
  res = movebaseClient(client, goal_pos, frame_id)
  if debug:
    print("return from movebaseClient:%d" %res)
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

def pointTransformations(pose, ang=0, cen=[0, 0], trans=[0, 0], scale=[1, 1]):
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

def showTextInRviz(marker_publisher, p, text, id, time, rgba):
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
  
def randomPose(r, ang):
  [x, y, z, w] = getCurrentXYZW()
  
  x, y = pointTransformations([x, y], ang, [x, y], [r, 0])
  
  theta = math.atan2(y, x)
  
  [qx, qy, z,w] = tf.transformations.quaternion_from_euler(0, 0, theta)
  
  return [x, y, z, w]
  
def getNextPose(marker_publisher):
  ang = np.random.randint(0, 360)
  _pose = randomPose(1.4, ang)
  newplan = getPlan(getCurrentXYZW(), _pose)
  replan = 1
  while replan:
    if (len(newplan) > 0):
      replan = 0
    else:
      ang = np.random.randint(0, 360)
      _pose = randomPose(1.4, ang)
      newplan = getPlan(getCurrentXYZW(), _pose)
  
  return _pose

MASK = []
is_scan = False
def processMapData():
  # CallStaticMapService()
  map_data = occupancyGridToNumpyArray(get_static_map())
  global MASK, is_scan
  img = (255-map_data).astype(np.uint8)
  img = cv2.flip(img, 0)
  ret, thresh = cv2.threshold(img.copy(),0,255,cv2.THRESH_BINARY)
  for cnt in MASK:
    cv2.drawContours(thresh, [cnt],0,255,-1)
  rgb = cv2.merge((thresh, thresh, thresh))
  # rgb = cv2.GaussianBlur(rgb,(3,3),0)
  im2,contours,hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  contours = sorted(contours, key=cv2.contourArea)
  contours = contours[:-1]
  if len(contours) > 0:
    cnt = contours[-1]
    M = cv2.moments(cnt)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    area = cv2.contourArea(cnt)
    cv2.drawContours(rgb,[cnt],0,(0,128,255),-1)
    print(area, (cX, cY))
    if not is_scan:
      MASK.append(cnt)
      is_scan = True
    print("============", len(contours))

  cv2.imshow("MAP", rgb)
  # cv2.imshow("MASK", thresh)

  cv2.waitKey(100)

def controlMain():
  client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
  
  pose_target = None

  r_pose = None
  rate = rospy.Rate(1.0)
  res = 0
  min_r = 1.2 # minimun_distance
  
  while not rospy.is_shutdown():
    try:
      if getCurrentXYZW()[0] != 0:
        # pose_target = getNextPose(marker_publisher)
        # showTextInRviz(marker_publisher, pose_target, 
          # "Next", 
          # 9, 
          # 10,
          # [0.2, 0.2, 0.8, 1.0])
        # res = moveToGoal(client, pose_target)
        # if res == 3:
          # showTextInRviz(marker_publisher, pose_target, 
          # "Done!!", 
          # 0, 
          # 2,
          # [0.2, 0.8, 0.2, 1.0])
        global is_scan
        if is_scan:
          is_scan = False
        rospy.sleep(5)
    except KeyboardInterrupt:
      break

def CallStaticMapService():
  try:
    # static_map_resp = rospy.ServiceProxy('/dynamic_map', nav_msgs.srv.GetMap)()
    # static_map_resp = rospy.ServiceProxy('/static_map', nav_msgs.srv.GetMap)()
    static_map = static_map_resp.map
    set_static_map(static_map)
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

def main():
  try:
    rospy.init_node('exploration_1')
    tf_listner = Thread(tfListenner)
    rospy.Subscriber("clicked_point", PointStamped, getClicked)
    rospy.Subscriber("/move_base/local_costmap/costmap", nav_msgs.msg.OccupancyGrid, costmapTopicListenner)
    rospy.Subscriber("/map", nav_msgs.msg.OccupancyGrid, mapTopicListenner)
    # CallStaticMapService()

    controlMain()
    #test()
    
  finally:
    tf_listner._Thread__stop()

if __name__ == '__main__':
  print "running random walk..."
  main()
  
