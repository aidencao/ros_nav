#!/usr/bin/env python
# coding:utf-8
import rospy
import GPS_Point
import os


from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from interactive_markers.menu_handler import *
from std_msgs.msg import String

menu_handler = MenuHandler()

# 定义全局变量
global a_lat, b_lat, a_lon, b_lon, a_x, b_x, a_y, b_y, point0, current_path, point_count, marker_size, box_size
current_path = False
point_count = 0


#用于获取水平面的高度
global horizontal_altitude
horizontal_altitude = 0
#用于保证只有刚开始打开时才能修改水平面高度
global flag 
flag = False

#移动灰色mark之后的操作
def moveFeedback(feedback):
    cleanPath()
    p = feedback.pose.position
    marker = server.get(feedback.marker_name)
    lat, lon = getGps(p.x, p.y)
    marker.description = "id:" + str(feedback.marker_name) + " x:"+str(p.x) + " y:" + \
        str(p.y) + " z:" + str(p.z) + " lat:" + str(lat) + "  lon:" + str(lon)
    server.insert(marker)
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


    print("createNewPoint----------------------------------0916")

    global server, menu_handler, point_count, marker_size, box_size

    int_marker = InteractiveMarker()
    int_marker.header.frame_id = frame_id
    int_marker.name = str(point_count)
    lat, lon = getGps(x, y)
    int_marker.description = "id:" + str(point_count) + " x:"+str(x) + " y:" + \
        str(y) + " z:" + str(5) + " lat:" + str(lat) + "  lon:" + str(lon)

    int_marker.pose.position.x = x
    int_marker.pose.position.y = y

    int_marker.pose.position.z = 5
    int_marker.scale = marker_size

    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = box_size
    box_marker.scale.y = box_size
    box_marker.scale.z = box_size
    box_marker.color.r = 1.0
    box_marker.color.g = 1.0
    box_marker.color.b = 1.0
    box_marker.color.a = 1.0

    box_control = InteractiveMarkerControl()
    box_control.interaction_mode = InteractiveMarkerControl.BUTTON
    box_control.always_visible = True
    box_control.markers.append(box_marker)

    int_marker.controls.append(box_control)

    move_control_x = InteractiveMarkerControl()
    move_control_x.orientation.w = 1
    move_control_x.orientation.x = 1
    move_control_x.orientation.y = 0
    move_control_x.orientation.z = 0
    normalizeQuaternion(move_control_x.orientation)
    move_control_x.name = "move_x"
    move_control_x.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

    int_marker.controls.append(move_control_x)

    move_control_y = InteractiveMarkerControl()
    move_control_y.orientation.w = 1
    move_control_y.orientation.x = 0
    move_control_y.orientation.y = 0
    move_control_y.orientation.z = 1
    normalizeQuaternion(move_control_y.orientation)
    move_control_y.name = "move_y"
    move_control_y.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

    int_marker.controls.append(move_control_y)

    move_control_z = InteractiveMarkerControl()
    move_control_z.orientation.w = 1
    move_control_z.orientation.x = 0
    move_control_z.orientation.y = 1
    move_control_z.orientation.z = 0
    normalizeQuaternion(move_control_z.orientation)
    move_control_z.name = "move_z"
    move_control_z.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

    int_marker.controls.append(move_control_z)

    server.insert(int_marker, moveFeedback)

    menu_handler.apply(server, str(point_count))

    server.applyChanges()
    point_count = point_count + 1


def showPointCallback(data):
    createNewPoint(data.header.frame_id, data.pose.position.x,
                   data.pose.position.y, data.header.seq)


# 获取当前位置
def setOdomCallback(data):
    global server, menu_handler, marker_size, box_size
    frame_id = data.header.frame_id
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z

    marker = server.get("-1")

    #为服务器创建一个交互式标记
    if marker == None:
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame_id
        int_marker.name = str(-1)
        lat, lon = getGps(x, y)
        int_marker.description = "id:" + str(-1) + " x:"+str(x) + " y:" + \
            str(y) + " z:" + str(z) + " lat:" + str(lat) + "  lon:" + str(lon)

        int_marker.pose.position.x = x
        int_marker.pose.position.y = y
        int_marker.pose.position.z = z
        int_marker.scale = marker_size



#定义一个盒子的对象，并设置其长宽高和颜色。
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = box_size
        box_marker.scale.y = box_size
        box_marker.scale.z = box_size
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0


    #创建一个包含框的非交互式控件
        box_control = InteractiveMarkerControl()
        box_control.interaction_mode = InteractiveMarkerControl.BUTTON
        box_control.always_visible = True



        box_control.markers.append(box_marker)

        int_marker.controls.append(box_control)

        server.insert(int_marker, moveFeedback)

        menu_handler.apply(server, str(-1))

        server.applyChanges()
    else:
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        lat, lon = getGps(x, y)
        marker.description = "id:" + str(-1) + " x:"+str(x) + " y:" + \
            str(y) + " z:" + str(z) + " lat:" + str(lat) + "  lon:" + str(lon)
        server.insert(marker)
        server.applyChanges()

# 通过GPS获取当前位置
# 通过GPS获取当前位置
def setGPSCallback(data):
    global flag 
    global horizontal_altitude

    if(data.status.status == -1):
        return;
    global server, menu_handler, marker_size, box_size
    lat = data.latitude
    lon = data.longitude

    #用于实地测试时的数据
    #z=data.altitude-610.516967
    #用于DJI仿真环境的测试数据
    
    #print("Z---z=data.altitude-610.516967")

    marker = server.get("-1")
    if marker == None:
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = str(-1)
        x, y = getXYZ(lat, lon)

        if flag == False:
            horizontal_altitude = data.altitude
            print("当前水平面高度为")
            print(horizontal_altitude)
            flag = True

        z = data.altitude - horizontal_altitude

        print("当前点的X、Y、Z坐标为")
        print(x,y,z)
        int_marker.description = "id:" + str(-1) + " x:"+str(x) + " y:" + \
            str(y) + " z:" + str(z) + " lat:" + str(lat) + "  lon:" + str(lon)

        int_marker.pose.position.x = x
        int_marker.pose.position.y = y
        int_marker.pose.position.z = z
        int_marker.scale = marker_size

        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = box_size
        box_marker.scale.y = box_size
        box_marker.scale.z = box_size
        #box_marker.color.r = 0.0
        #box_marker.color.g = 0.5
        #box_marker.color.b = 0.5
        #box_marker.color.a = 1.0


        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 1.0
        box_marker.color.a = 1.0
        

        box_control = InteractiveMarkerControl()
        box_control.interaction_mode = InteractiveMarkerControl.BUTTON
        box_control.always_visible = True
        box_control.markers.append(box_marker)

        int_marker.controls.append(box_control)

        server.insert(int_marker, moveFeedback)

        menu_handler.apply(server, str(-1))

        server.applyChanges()
    else:
        x, y = getXYZ(lat, lon)
        z = data.altitude - horizontal_altitude
        #print(x,y)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.description = "id:" + str(-1) + " x:"+str(x) + " y:" + \
            str(y) + " z:" + str(z) + " lat:" + str(lat) + "  lon:" + str(lon)
        server.insert(marker)
        server.applyChanges()


# def menuDeleteCallback(feedback):  # 删除当前点
#     rospy.loginfo(feedback.marker_name)
#     server.erase(feedback.marker_name)
#     server.applyChanges()


def menuResetPointsCallback(data):  # 清空巡航点
    global point_count
    point_count = 0
    server.clear()
    server.applyChanges()
    cleanPath()


def menuShowPointsCallback(feedback):
    for key in sorted(server.marker_contexts):
        marker = server.marker_contexts[key].int_marker
        print("编号：" + key + " x:" + str(marker.pose.position.x) + " y:" +
              str(marker.pose.position.y) + " z:" +
              str(marker.pose.position.z) )
        


        
    print("\n")


def cleanPath():  # 清空路径
    global current_path

    if current_path != False:
        nav_path = Path()
        nav_path.header.stamp = rospy.Time.now()
        nav_path.header.frame_id = "map"
        traj_pub.publish(nav_path)
        current_path = False


# def menuSetStartCallback(feedback):
#     global current_path
#     p = feedback.pose.position

#     point = PointStamped()
#     point.header.frame_id = "map"
#     point.header.stamp = rospy.Time.now()
#     point.point.x = p.x
#     point.point.y = p.y
#     point.point.z = p.z

#     current_path = False

#     start_pub.publish(point)

#     print feedback.marker_name + " is now at " + \
#         str(p.x) + ", " + str(p.y) + ", " + str(p.z) + ", 设为起点"

# def menuSetGoalCallback(feedback):
#     global current_path
#     p = feedback.pose.position

#     point = PointStamped()
#     point.header.frame_id = "map"
#     point.header.stamp = rospy.Time.now()
#     point.point.x = p.x
#     point.point.y = p.y
#     point.point.z = p.z

#     current_path = False

#     goal_pub.publish(point)

#     print feedback.marker_name + " is now at " + \
#         str(p.x) + ", " + str(p.y) + ", " + str(p.z) + ", 设为终点"


def initMenu():
    # menu_handler.insert("删除", callback=menuDeleteCallback)
    menu_handler.insert("重置巡航点", callback=menuResetPointsCallback)
    menu_handler.insert("显示巡航点信息", callback=menuShowPointsCallback)
    menu_handler.insert("发送巡航点，进行路径规划", callback=menuSendPointsCallback)
    # menu_handler.insert("设为导航起点", callback=menuSetStartCallback)
    # menu_handler.insert("设为导航终点", callback=menuSetGoalCallback)



    menu_handler.insert("转换为GPS路径，通过串口发送", callback=sendGpsWaypoints)
    menu_handler.insert("通过串口直接发送地图坐标", callback=sendXYZWaypoints)

    menu_handler.insert("GPS激活", callback=menuActivation)


##############################################################################

# def x_latitudeMapping(x1, lat1, x2, lat2):  # 纬度向北增加 经度往东增加 假定x轴与纬度重合
#     a = (lat1 - lat2) / (x1 - x2)
#     b = lat1 - a * x1
#     return a, b

# def y_longitudeMapping(y1, lon1, y2, lon2):  # 经度向东增加 假定y轴负方向与经度重合
#     a = (lon1 - lon2) / (y1 - y2)
#     b = lon1 - a * y1
#     return a, b


def getGps(x, y):
    # global a_lat, b_lat, a_lon, b_lon
    # lat = a_lat * x + b_lat
    # lon = a_lon * y + b_lon
    # return lat, lon
    global a_lat, b_lat, a_lon, b_lon
    lat = point0.getLat() + a_lat * (x - point0.getx()) + b_lat * (
        y - point0.gety())
    lon = point0.getLon() + a_lon * (x - point0.getx()) + b_lon * (
        y - point0.gety())

    return lat, lon


def getXYZ(lat, lon):
    global a_x, b_x, a_y, b_y
    x = point0.getx() + a_x * (lat-point0.getLat()) + b_x * (lon-point0.getLon())
    y = point0.gety() + a_y * (lat-point0.getLat()) + b_y * (lon-point0.getLon())

    return x, y


def checkParam(x1, x2, y1, y2, lat1, lat2, lon1, lon2):
    if (x1 != x2) and (y1 != y2) and (lat1 != lat2) and (lon1 != lon2):
        return True
    else:
        return False


def transGPS(path):  # 将规划出的路径转换为GPS信息
    waypoints_divid = "$"
    waypoints_seq = []

    for i, point in enumerate(path.poses[1:]):
        gps_divid = "@"
        lat, lon = getGps(point.pose.position.x, point.pose.position.y)
        gps_point_seq = (str(lat), str(lon), str(point.pose.position.z))
        
        #print("---------test8.3------lat:" + str(lat) + "  lon:" + str(lon))


        waypoints_seq.append(gps_divid.join(gps_point_seq))

    return waypoints_divid.join(waypoints_seq)


def getPathCallback(data):
    global current_path
    if data.poses == []:
        current_path = False
    else:
        current_path = data


        

# 测试GPS映射是否满意

def testGpsMapping():
    points = GPS_Point.readPoint(
        "/home/cjz/nav_ws/src/gps_mapping/src/mark_7.14.txt")

    for p in points:
        lat, lon = getGps(p.getx(), p.gety())
        print("x:" + str(p.getx()) + "  y:" + str(p.gety())+" lat:" +
              str(lat-p.getLat()) + "  lon:" + str(lon-p.getLon()))

# 测试反向映射是否满意


def testXYZMapping():
    points = GPS_Point.readPoint(
        "/home/cjz/nav_ws/src/gps_mapping/src/mark_7.14.txt")

    for p in points:
        x, y = getXYZ(p.getLat(), p.getLon())
        print("lat:" + str(p.getLat()) + "  lon:" + str(p.getLon())+" x:" +
              str(x-p.getx()) + "  y:" + str(y-p.gety()))

###############################################################
# 用于多点位导航标记


def setHighCallback(data):  # 测试直接设置高度的代码
    # pose = data.pose
    # pose.position.z = 10
    # server.setPose(data.marker_name, pose)

    # marker = server.get(data.marker_name)
    # marker.description = "xxxxxxxxxxxxxx"
    # server.insert(marker)
    # server.applyChanges()

    msg = data.data.split("$", 1)
    point_id = msg[0]
    height = msg[1]

    marker = server.get(point_id)
    if marker == None:
        rospy.logwarn("不存在对应编号的路径点")
        return

    try:
        heightd = float(height)
    except ValueError:
        rospy.logwarn("输入高度非法")
        return

    marker.pose.position.z = heightd
    lat, lon = getGps(marker.pose.position.x, marker.pose.position.y)
    marker.description = "id:" + str(point_id) + " x:"+str(marker.pose.position.x) + " y:" + \
        str(marker.pose.position.y) + " z:" + str(heightd) + \
        " lat:" + str(lat) + "  lon:" + str(lon)
    server.insert(marker)
    server.applyChanges()


def menuSendPointsCallback(data):  # 发送巡航点
    # 生成巡航点路径消息
    nav_path = Path()
    nav_path.header.stamp = rospy.Time.now()
    nav_path.header.frame_id = "map"

    for key in sorted(server.marker_contexts):



        marker = server.marker_contexts[key].int_marker


        pose = PoseStamped()



        pose.pose.position.x = marker.pose.position.x
        pose.pose.position.y = marker.pose.position.y
        pose.pose.position.z = marker.pose.position.z

        pose.pose.orientation.x = marker.pose.orientation.x
        pose.pose.orientation.y = marker.pose.orientation.y
        pose.pose.orientation.z = marker.pose.orientation.z
        pose.pose.orientation.w = marker.pose.orientation.w

        nav_path.poses.append(pose)

    nav_pub.publish(nav_path)      #发布消息
    cleanPath()


#########################################################################################
# 用于与串口程序通信


def sendGpsWaypoints(data):
    global current_path

    if current_path != False:
        waypoints_msg = transGPS(current_path)
        gps_pub.publish(waypoints_msg)
        print("sendGpsWaypoints 转换出来的path为" )      
    else:
        rospy.logwarn("当前还未有已生成的路径")


def sendXYZWaypoints(data):
    global current_path

    if current_path != False:
        xyz_pub.publish(current_path)
    else:
        rospy.logwarn("当前还未有已生成的路径")




def menuActivation(data):
        a = "Activation" #用于触发激活GPS功能
        Activation_pub.publish(a)
        print("menuActivation 成功执行")




if __name__ == '__main__':
    try:
        rospy.init_node("point_marker_node")
        rospy.loginfo("Starting point_marker")



        #参数都可以通过launch文件修改
        # 获取标定点参数
        x1 = rospy.get_param("gps_mapping_node/x1", 2)
        y1 = rospy.get_param("gps_mapping_node/y1", 2)
        x2 = rospy.get_param("gps_mapping_node/x2", 2)
        y2 = rospy.get_param("gps_mapping_node/y2", 2)
        lat1 = rospy.get_param("gps_mapping_node/lat1", 2)
        lon1 = rospy.get_param("gps_mapping_node/lon1", 2)
        lat2 = rospy.get_param("gps_mapping_node/lat2", 2)
        lon2 = rospy.get_param("gps_mapping_node/lon2", 2)






        marker_size = rospy.get_param("gps_mapping_node/marker_size", 4.5)
        box_size = rospy.get_param("gps_mapping_node/box_size", 3)



        file_path = rospy.get_param(
            "gps_mapping_node/file_path", "/home/cjz/nav_ws/src/gps_mapping/src/test_7.14.txt")

            #"gps_mapping_node/file_path", "${workspaceFolder}/src/gps_mapping/src/test.txt"
	







        # 判断标定点是否合法      数据手动输入，一定合法，后续可考虑改进，目前不需要。
        if checkParam(x1, x2, y1, y2, lat1, lat2, lon1, lon2):
            #     a_lat, b_lat = x_latitudeMapping(x1, lat1, x2, lat2)
            #     a_lon, b_lon = y_longitudeMapping(y1, lon1, y2, lon2)
            a_lat, b_lat, a_lon, b_lon, a_x, b_x, a_y, b_y, point0 = GPS_Point.avg_mapping(GPS_Point.readPoint(file_path))






            # 关注标点相关主题
            rospy.Subscriber("goal",
                             PoseStamped,
                             showPointCallback,
                             queue_size=1)

            rospy.Subscriber("nav/waypoints",
                             Path,
                             getPathCallback,
                             queue_size=1)

            rospy.Subscriber("gps_point_cmd",
                             String,
                             setHighCallback,
                             queue_size=1)

            rospy.Subscriber("odom", Odometry, setOdomCallback, queue_size=1)
            
            
     


            rospy.Subscriber("raw_fix", NavSatFix, setGPSCallback, queue_size=1)

            # start_pub = rospy.Publisher(
            #     'nav/start', PointStamped, queue_size=1)
            # goal_pub = rospy.Publisher('nav/goal', PointStamped, queue_size=1)



            gps_pub = rospy.Publisher("gps/path", String,
                                      queue_size=1)  # 用于向串口发送GPS路径



 
            nav_pub = rospy.Publisher("path/waypoints", Path,
                                      queue_size=1)  # 发送路径规划锚点



            traj_pub = rospy.Publisher("nav/waypoints", Path,
                                       queue_size=1)  # 用于将规划好的路径清空




            xyz_pub = rospy.Publisher("xyz/path", Path,
                                      queue_size=1)  # 用于向串口发送地图坐标路径

            
            Activation_pub = rospy.Publisher("Activation",String,
                                        queue_size=1)   #用于激活GPS


            





            # 创建交互式标记服务，命名空间为nav_points
            server = InteractiveMarkerServer("nav_points")

            initMenu()

            testGpsMapping()
            testXYZMapping()

            # 关闭前清除标记
            rospy.on_shutdown(cleanPath)

            rospy.spin()
        else:
            rospy.logerr("输入参数非法")
    except KeyboardInterrupt:
        print("Shutting down point_marker")
        
