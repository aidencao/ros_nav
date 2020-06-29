#!/usr/bin/env python
# coding:utf-8
import GPS_Point
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

x, y, lat, lon


def poseCallback(data):
    x = data.pose.position.x
    y = data.pose.position.y


def gpsCallback(data):
    lat = data.latitude
    lon = data.longitude


def writeFile(path):
    with open(path, 'a+') as f:
        f.write(str(x)+','+str(y)+'     '+str(
            lat)+','+str(lon)+'\n')


if __name__ == '__main__':
    try:
        rospy.init_node("GPS_mark_node")
        rospy.loginfo("Starting GPS_mark")

        file_path = rospy.get_param(
            "GPS_mark_node/file_path", "/home/cyr/nav_ws/src/gps_mapping/src/testmark.txt")
        pose_topic = rospy.get_param(
            "GPS_mark_node/pose_topic", "/current_pose")
        gps_topic = rospy.get_param(
            "GPS_mark_node/gps_topic", "/raw_fix")
        write_topic = rospy.get_param(
            "GPS_mark_node/write_topic", "/write")    

        rospy.Subscriber(pose_topic,
                         PoseStamped,
                         poseCallback,
                         queue_size=1)

        rospy.Subscriber(gps_topic,
                         NavSatFix,
                         gpsCallback,
                         queue_size=1)
        
        rospy.Subscriber(write_topic,
                         String,
                         gpsCallback,
                         queue_size=1)



    except KeyboardInterrupt:
        print("Shutting down GPS_mark")
