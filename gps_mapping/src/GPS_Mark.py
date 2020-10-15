#!/usr/bin/env python
# coding:utf-8
import GPS_Point
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

x, y, lat, lon



#计数器 用于间隔取点
global count=1



#用于获取时间戳转换出来的浮点数和计算时间间隔
global float time_mark1
global float time_mark2

global float time_dif






def poseCallback(data):
    x = data.pose.position.x
    y = data.pose.position.y
    
    time_mark1=data.header.stamp.toSec()


    count=count + 1

    time_dif=float(time_mark1)-float(time_mark2)

    if count%10 == 0:
        int i=0
        while time_dif>0.1 or time_dif<-0.1:
                time.sleep(1)
                i=i+1
                if i==300: time_dif=0.0

        with open(rospy.get_param("GPS_mark_node/gps_topic", "/raw_fix"), 'a+') as f:
                f.write(str(x)+','+str(y)+'     '+str(lat)+','+str(lon)+'\n')



    with open(rospy.get_param("GPS_mark_node/gps_topic", "/raw_fix_position"), 'a+') as f:
                f.write(str(x)+','+str(y)+'     '+str(lat)+','+str(lon)+'\n')

#gps数据获取更慢

def gpsCallback(data):
    lat = data.latitude
    lon = data.longitude


    time_mark2=data.header.stamp.toSec()

        with open(rospy.get_param("GPS_mark_node/gps_topic", "/gps"), 'a+') as f:
                f.write(str(x)+','+str(y)+'     '+str(lat)+','+str(lon)+'\n')

def writeFile(path):
    with open(path, 'a+') as f:
        f.write(str(x)+','+str(y)+'     '+str(
            lat)+','+str(lon)+'\n')


if __name__ == '__main__':
    try:
        rospy.init_node("GPS_mark_node")
        rospy.loginfo("Starting GPS_mark")

        file_path = rospy.get_param(
            "GPS_mark_node/file_path", "/home/cjz/nav_ws/src/gps_mapping/src/testmark.txt")

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
