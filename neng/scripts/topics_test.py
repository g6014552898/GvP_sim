#!/usr/bin/env python
import message_filters
import rospy, sys
from sensor_msgs.msg import Image, PointCloud2

def image_cb(depth_img, rgb_img, cloud):
    global subs, counter, storage, rgb_publisher, depth_publisher, cloud_publisher
    # storage[counter] = [depth_img, rgb_img, cloud]
    
    depth_publisher.publish(depth_img)
    rgb_publisher.publish(rgb_img)
    cloud_publisher.publish(cloud)
    
    counter += 1
    if counter >= 3:
      # [sub.sub.unregister() for sub in subs]
      print("Done")
      # rospy.signal_shutdown("Exit")
      rospy.sleep(5)
      counter = 0
def init_system():
  global subs, counter, storage, rgb_publisher, depth_publisher, cloud_publisher
  counter = 0
  rospy.init_node("topics_test")
  depth_sub = message_filters.Subscriber("/camera/depth/image_raw", Image)
  rgb_sub = message_filters.Subscriber("/camera/rgb/image_raw", Image)
  cloud_sub = message_filters.Subscriber("/camera/depth/points", PointCloud2)

  rgb_publisher = rospy.Publisher('/good/depth/image_raw', Image, queue_size=1)
  depth_publisher = rospy.Publisher('/good/rgb/image_raw', Image, queue_size=1)
  cloud_publisher = rospy.Publisher('/good/depth/points', PointCloud2, queue_size=1)

  subs = [depth_sub, rgb_sub, cloud_sub]
  storage = [[], [], []]
  ts = message_filters.ApproximateTimeSynchronizer(subs, 10, 1)
  ts.registerCallback(image_cb)

init_system()
rospy.spin()

# for data in storage:
  # print data[0].header
  # print data[1].header
  # print data[2].header
  # print "======================"
  
# sys.exit(1)