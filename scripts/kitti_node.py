#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import os
import cv2
from cv_bridge import CvBridge
import rospy
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import Image,PointCloud2
import sensor_msgs.point_cloud2 as pcl2

BASE_PATH = "/home/mckros/kitti/RawData/2011_09_26/2011_09_26_drive_0005_sync/"    

if __name__ == "__main__":
    rospy.init_node("kitti_node",anonymous=True)
    cam_color_left_pub  = rospy.Publisher("/kitti/cam_color_left", Image, queue_size=10)
    # cam_color_right_pub = rospy.Publisher("/kitti/cam_color_right", Image, queue_size=10)
    # cam_gray_left_pub   = rospy.Publisher("/kitti/cam_gray_left", Image, queue_size=10)
    # cam_gray_lright_pub = rospy.Publisher("/kitti/cam_gray_right", Image, queue_size=10)
    pcl_pub = rospy.Publisher("/kitti/pointcloud2", PointCloud2, queue_size=10)
    bridge = CvBridge()

    rate = rospy.Rate(10)
    num = 1
    while not rospy.is_shutdown():
        img = cv2.imread(os.path.join(BASE_PATH, "image_02/data/%010d.png"%num))
        point_cloud =np.fromfile(os.path.join(BASE_PATH, "velodyne_points/data/%010d.bin"%num), dtype=np.float32).reshape(-1,4)
        cam_color_left_pub.publish(bridge.cv2_to_imgmsg(img,"bgr8"))
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        pcl_pub.publish(pcl2.create_cloud_xyz32(header,point_cloud[:,:3]))
        rospy.loginfo("kitti published")
        rate.sleep()
        num+=1
        num%=154
