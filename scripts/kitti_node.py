#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import os
import rospy
import cv2
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, Imu, NavSatFix
import sensor_msgs.point_cloud2 as pcl2
from tf import transformations
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import pandas as pd

BASE_PATH = "/home/mckros/kitti/RawData/2011_09_26/2011_09_26_drive_0005_sync/"   
# https://github.com/pratikac/kitti/blob/master/readme.raw.txt
IMU_NAME = ["lat", "lon", "alt", "roll", "pitch", "yaw", "vn", "ve", "vf", "vl", "vu", "ax", "ay", "az", "af", "al", "au", "wx", "wy", "wz", "wf", "wl", "wu", "posacc", "velacc", "navstat", "numsats", "posmode", "velmode", "orimode"]
# https://blog.csdn.net/xiaokai1999/article/details/119393275
LABEL_NAME = ["frame", "track id", "type", "truncated", "occluded", "alpha", "bbox_left", "bbox_top", "bbox_right", "bbox_bottom", "dimensions_height", "dimensions_width", "dimensions_length", "location_x", "location_y", "location_z", "rotation_y"]

#----- 相机数据发布函数 -----#
def publish_cam_fun(num,df,cam_pub):
    bridge = CvBridge()
    img = cv2.imread(os.path.join(BASE_PATH, "image_02/data/%010d.png"%num))
    boxs = np.array(df.loc[df.frame.isin([num]),['type','bbox_left','bbox_top','bbox_right','bbox_bottom']])
    for box in boxs:
        left_point = (int(box[1]),int(box[2]))
        right_point = (int(box[3]),int(box[4]))
        if box[0] == 'Car':
            cv2.rectangle(img,left_point,right_point,(255,0,0),2)
        elif box[0] == 'Cyclist':
            cv2.rectangle(img,left_point,right_point,(0,255,0),2)
        elif box[0] == 'Pedestrian':
            cv2.rectangle(img,left_point,right_point,(0,0,255),2)
    cam_pub.publish(bridge.cv2_to_imgmsg(img,"bgr8"))

#----- 点云数据发布函数 -----#
def publish_pcl_fun(num, pcl_pub):
    point_cloud =np.fromfile(os.path.join(BASE_PATH, "velodyne_points/data/%010d.bin"%num), dtype=np.float32).reshape(-1,4)
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    pcl_pub.publish(pcl2.create_cloud_xyz32(header,point_cloud[:,:3]))

#----- 相机视野Marker数据发布函数 -----#
def publish_cam_angle_line_fun(cam_angle_line_pub):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()

    # 每个显示的marker都需要不一样的id，否则会覆盖
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.lifetime = rospy.Duration()# 永久显示
    
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0 # 透明度
    marker.scale.x = 0.2 # 线粗细

    # 这边点的数据主要看官方提供的位置图
    # 会在每两个连续点之间画一条线0-1，1-2。。。
    marker.points = []
    marker.points.append(Point(10,-10,0)) 
    marker.points.append(Point(0,0,0))
    marker.points.append(Point(10,10,0))

    cam_angle_line_pub.publish(marker)

#----- 自车模型数据发布函数 -----#
def publish_car_model(car_model_pub):
    mech_marker = Marker()
    mech_marker.header.frame_id = "map"
    mech_marker.header.stamp = rospy.Time.now()

    mech_marker.id = -1
    mech_marker.type = Marker.MESH_RESOURCE
    mech_marker.action = Marker.ADD
    mech_marker.lifetime = rospy.Duration()
    mech_marker.mesh_resource = "package://kitti_tutorials/meshes/Car.dae"

    # 位置主要看官方提供的位置图
    mech_marker.pose.position.x = 0.0
    mech_marker.pose.position.y = 0.0
    mech_marker.pose.position.z = -1.73

    q = transformations.quaternion_from_euler(np.pi,np.pi,-np.pi/2.0)
    mech_marker.pose.orientation.x = q[0]
    mech_marker.pose.orientation.y = q[1]
    mech_marker.pose.orientation.z = q[2]
    mech_marker.pose.orientation.w = q[3]

    mech_marker.color.r = 1.0
    mech_marker.color.g = 1.0
    mech_marker.color.b = 1.0
    mech_marker.color.a = 1.0 # 透明度

    mech_marker.scale.x = 1.0 # 线粗细
    mech_marker.scale.y = 1.0 # 线粗细
    mech_marker.scale.z = 1.0 # 线粗细
    
    car_model_pub.publish(mech_marker)

#----- 读取imu，gps数据 -----#
def read_oxtsfile(num):
    data = pd.read_csv(os.path.join(BASE_PATH, "oxts/data/%010d.txt"%num), header=None, sep=" ")
    data.columns = IMU_NAME
    return data

#----- 读取图片标注数据,并做相应的处理 -----#
def read_labelfile():
    data = pd.read_csv(os.path.join(BASE_PATH, "label_02/0000.txt"), header=None, sep=" ")
    data.columns = LABEL_NAME
    data.loc[data.type.isin(['Van','Car','Truck']),'type'] = 'Car'
    data = data[data.type.isin(['Car','Cyclist','Pedestrian'])]
    return data

#----- 自车imu数据发布函数,sudo apt install ros-noetic-rviz-imu-plugin  -----#
def publish_imu(imu_data,imu_pub):
    imu = Imu()

    imu.header.frame_id = "map"
    imu.header.stamp = rospy.Time.now()

    q = transformations.quaternion_from_euler(float(imu_data.roll), float(imu_data.pitch), float(imu_data.yaw))
    imu.orientation.x = q[0]
    imu.orientation.y = q[1]
    imu.orientation.z = q[2]
    imu.orientation.w = q[3]

    imu.linear_acceleration.x = imu_data.af
    imu.linear_acceleration.y = imu_data.al
    imu.linear_acceleration.z = imu_data.au

    imu.angular_velocity.x = imu_data.wx
    imu.angular_velocity.y = imu_data.wy
    imu.angular_velocity.z = imu_data.wz

    imu_pub.publish(imu)

#----- 自车gps数据发布函数 -----#
def publish_gps(gps_data, gps_pub):
    gps = NavSatFix()

    gps.header.frame_id = "map"
    gps.header.stamp = rospy.Time.now()

    gps.latitude = gps_data.lat
    gps.longitude = gps_data.lon
    gps.altitude = gps_data.alt

    gps_pub.publish(gps)


if __name__ == "__main__":
    rospy.init_node("kitti_node",anonymous=True)
    cam_color_left_pub  = rospy.Publisher("/kitti/cam_color_left", Image, queue_size=10)
    pcl_pub = rospy.Publisher("/kitti/pointcloud2", PointCloud2, queue_size=10)
    cam_angle_line_pub = rospy.Publisher("/kitti/cam_angle_line", Marker, queue_size=10)
    car_model_pub = rospy.Publisher("/kitti/car_model", Marker, queue_size=10)
    imu_pub = rospy.Publisher("/kitti/imu", Imu, queue_size=10)
    gps_pub = rospy.Publisher("/kitti/gps", NavSatFix, queue_size=10)
    df = read_labelfile()
    rate = rospy.Rate(10)
    num = 1
    while not rospy.is_shutdown():
        publish_cam_fun(num, df, cam_color_left_pub)
        publish_pcl_fun(num, pcl_pub)
        publish_cam_angle_line_fun(cam_angle_line_pub)
        publish_car_model(car_model_pub)
        data = read_oxtsfile(num)
        publish_imu(data, imu_pub)
        publish_gps(data, gps_pub)
        rospy.loginfo("kitti published")

        rate.sleep()
        num+=1
        num%=154
