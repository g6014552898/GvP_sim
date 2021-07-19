#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np

# rospy.wait_for_message('/good/depth/points', PointCloud2) 

def init_subscribers():
    rospy.Subscriber('/good/depth/points', PointCloud2, on_new_point_cloud)

def on_new_point_cloud(data):
    assert isinstance(data, PointCloud2)
    gen = pc2.read_points(data)
    print len(data.data)

    
def listener():
    rospy.init_node('pcl_test_1', anonymous=True)
    init_subscribers()
    rospy.spin()
    

if __name__ == "__main__":
    listener()