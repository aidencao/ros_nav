#!/usr/bin/env python
# coding:utf-8
import rospy
# from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped


def x_latitudeMapping(x1, lat1, x2, lat2):  # 纬度向北增加 经度往东增加 假定x轴与纬度重合
    a = (lat1-lat2)/(x1-x2)
    b = lat1-a*x1
    return a, b


def y_longitudeMapping(y1, lon1, y2, lon2):  # 经度向东增加 假定y轴负方向与经度重合
    a = (lon1-lon2)/(y1-y2)
    b = lon1-a*y1
    return a, b


def getGps(x, y):
    global a_lat, b_lat, a_lon, b_lon
    lat = a_lat*x+b_lat
    lon = a_lon*y+b_lon
    return lat, lon


def checkParam(x1, x2, y1, y2, lat1, lat2, lon1, lon2):
    if (x1 != x2) and (y1 != y2) and (lat1 != lat2) and (lon1 != lon2):
        return True
    else:
        return False


def transGPSCallback(data):
    lat, lon = getGps(data.point.x, data.point.y)
    rospy.loginfo("x:" + str(data.point.x) + "  y:" +
                  str(data.point.y)+" lat:" + str(lat) + "  lon:" + str(lon))


def testFunc():  # 用于测试功能，固定发布标点信息
    point_pub = rospy.Publisher(
        "point_in_map", PointStamped, queue_size=10)

    point = PointStamped()
    point.header.frame_id = "camera"
    point.header.stamp = rospy.Time.now()
    point.point.x = 3.73646950722
    point.point.y = -35.8251457214
    point.point.z = 0

    loop_rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        point_pub.publish(point)
        rospy.loginfo("send point_marker")
        loop_rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node("gps_mapping_node")
        rospy.loginfo("Starting gps_mapping")
        # 定义全局变量
        global a_lat, b_lat, a_lon, b_lon

        # 获取标定点参数
        x1 = rospy.get_param("gps_mapping_node/x1", 2)
        y1 = rospy.get_param("gps_mapping_node/y1", 2)
        x2 = rospy.get_param("gps_mapping_node/x2", 2)
        y2 = rospy.get_param("gps_mapping_node/y2", 2)
        lat1 = rospy.get_param("gps_mapping_node/lat1", 2)
        lon1 = rospy.get_param("gps_mapping_node/lon1", 2)
        lat2 = rospy.get_param("gps_mapping_node/lat2", 2)
        lon2 = rospy.get_param("gps_mapping_node/lon2", 2)

        # 判断标定点是否合法
        if checkParam(x1, x2, y1, y2, lat1, lat2, lon1, lon2):
            a_lat, b_lat = x_latitudeMapping(x1, lat1, x2, lat2)
            a_lon, b_lon = y_longitudeMapping(y1, lon1, y2, lon2)

            # 关注转换点位主题
            rospy.Subscriber("point_in_map", PointStamped,
                             transGPSCallback, queue_size=10)
            # 启动测试
            testFunc()
            rospy.spin()
        else:
            rospy.logerr("输入参数非法")
    except KeyboardInterrupt:
        print("Shutting down gps_mapping")
