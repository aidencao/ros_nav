#!/usr/bin/env python
# coding:utf-8
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from interactive_markers.menu_handler import *

menu_handler = MenuHandler()


def moveFeedback(feedback):
    p = feedback.pose.position
    print feedback.marker_name + " is now at " + \
        str(p.x) + ", " + str(p.y) + ", " + str(p.z)
    server.applyChanges()


def normalizeQuaternion(quaternion_msg):
    norm = quaternion_msg.x**2 + quaternion_msg.y**2 + \
        quaternion_msg.z**2 + quaternion_msg.w**2
    s = norm**(-0.5)
    quaternion_msg.x *= s
    quaternion_msg.y *= s
    quaternion_msg.z *= s
    quaternion_msg.w *= s


def createNewPoint(frame_id, x, y, seq):
    global server, menu_handler, point_count

    int_marker = InteractiveMarker()
    int_marker.header.frame_id = frame_id
    int_marker.name = str(point_count)
    int_marker.description = str(point_count)
    int_marker.pose.position.x = x
    int_marker.pose.position.y = y

    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.45
    box_marker.scale.y = 0.45
    box_marker.scale.z = 0.45
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    box_control = InteractiveMarkerControl()
    box_control.interaction_mode = InteractiveMarkerControl.BUTTON
    box_control.always_visible = True
    box_control.markers.append(box_marker)

    int_marker.controls.append(box_control)

    move_control = InteractiveMarkerControl()
    move_control.orientation.w = 1
    move_control.orientation.x = 0
    move_control.orientation.y = 1
    move_control.orientation.z = 0
    normalizeQuaternion(move_control.orientation)
    move_control.name = "move_z"
    move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

    int_marker.controls.append(move_control)

    server.insert(int_marker, moveFeedback)

    menu_handler.apply(server, str(point_count))

    server.applyChanges()
    point_count = point_count + 1


def showPointCallback(data):
    createNewPoint(data.header.frame_id,
                   data.pose.position.x, data.pose.position.y, data.header.seq)

# 删除当前点
def menuDeleteCallback(feedback):
    rospy.loginfo(feedback.marker_name)
    server.erase(feedback.marker_name)
    server.applyChanges()

# 清空巡航点
def menuResetPointsCallback(data):
    global point_count
    point_count = 0
    server.clear()
    server.applyChanges()

def menuShowPointsCallback(feedback):
    for key in sorted(server.marker_contexts):
        marker = server.marker_contexts[key].int_marker
        print("编号：" + key + " x:"+str(marker.pose.position.x) + " y:" +
              str(marker.pose.position.y) + " z:"+str(marker.pose.position.z))
    print("\n")


def menuSetStartCallback(feedback):
    global current_path
    p = feedback.pose.position

    point = PointStamped()
    point.header.frame_id = "camera"
    point.header.stamp = rospy.Time.now()
    point.point.x = p.x
    point.point.y = p.y
    point.point.z = p.z

    current_path = False

    start_pub.publish(point)

    print feedback.marker_name + " is now at " + \
        str(p.x) + ", " + str(p.y) + ", " + str(p.z) + ", 设为起点"


def menuSetGoalCallback(feedback):
    global current_path
    p = feedback.pose.position

    point = PointStamped()
    point.header.frame_id = "camera"
    point.header.stamp = rospy.Time.now()
    point.point.x = p.x
    point.point.y = p.y
    point.point.z = p.z

    current_path = False

    goal_pub.publish(point)

    print feedback.marker_name + " is now at " + \
        str(p.x) + ", " + str(p.y) + ", " + str(p.z) + ", 设为终点"


def initMenu():
    # menu_handler.insert("删除", callback=menuDeleteCallback)
    menu_handler.insert("重置巡航点", callback=menuResetPointsCallback)
    menu_handler.insert("显示巡航点信息", callback=menuShowPointsCallback)
    menu_handler.insert("设为导航起点", callback=menuSetStartCallback)
    menu_handler.insert("设为导航终点", callback=menuSetGoalCallback)
    menu_handler.insert("生成GPS路径", callback=transGPSCallback)
    menu_handler.insert("高度设为10", callback=testSet10)


##############################################################################
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
    global current_path

    if current_path != False:
        for i, point in enumerate(current_path):
            lat, lon = getGps(point.pose.position.x, point.pose.position.y)
            rospy.loginfo("x:" + str(point.pose.position.x) + "  y:" + str(point.pose.position.y) +
                          "  z:" + str(point.pose.position.z)+" lat:" + str(lat) + "  lon:" + str(lon))
    else:
        rospy.logwarn("当前还未有已生成的路径")


def getPathCallback(data):
    global current_path
    current_path = data.poses

###############################################################
# 用于多点位导航标记

# 测试直接设置高度的代码
def testSet10(data):
    pose = data.pose
    pose.position.z = 10
    server.setPose(data.marker_name, pose)
    server.applyChanges()


if __name__ == '__main__':
    try:
        rospy.init_node("point_marker_node")
        rospy.loginfo("Starting point_marker")

        # 定义全局变量
        global a_lat, b_lat, a_lon, b_lon, current_path, point_count
        current_path = False
        point_count = 0

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

            # 关注标点相关主题
            rospy.Subscriber("goal", PoseStamped,
                             showPointCallback, queue_size=1)

            rospy.Subscriber("waypoints", Path, getPathCallback, queue_size=1)

            start_pub = rospy.Publisher(
                'nav/start', PointStamped, queue_size=1)
            goal_pub = rospy.Publisher('nav/goal', PointStamped, queue_size=1)

            # 创建交互式标记服务，命名空间为nav_points
            server = InteractiveMarkerServer("nav_points")

            initMenu()

            rospy.spin()
        else:
            rospy.logerr("输入参数非法")
    except KeyboardInterrupt:
        print("Shutting down point_marker")
