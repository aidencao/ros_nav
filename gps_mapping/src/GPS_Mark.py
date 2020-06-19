#!/usr/bin/env python
# coding:utf-8
import GPS_Point

if __name__ == '__main__':
    try:
        rospy.init_node("point_marker_node")
        rospy.loginfo("Starting point_marker")

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
        file_path = rospy.get_param("gps_mapping_node/file_path","/home/cyr/nav_ws/src/gps_mapping/src/test.txt")

        # 判断标定点是否合法
        if checkParam(x1, x2, y1, y2, lat1, lat2, lon1, lon2):
            #     a_lat, b_lat = x_latitudeMapping(x1, lat1, x2, lat2)
            #     a_lon, b_lon = y_longitudeMapping(y1, lon1, y2, lon2)
            a_lat, b_lat, a_lon, b_lon, point0 = GPS_Point.avg_mapping(
                GPS_Point.readPoint(
                    file_path))

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
                                      queue_size=1)  #用于向串口发送地图坐标路径

            # 创建交互式标记服务，命名空间为nav_points
            server = InteractiveMarkerServer("nav_points")

            initMenu()

            lat, lon = getGps(-10.1541414261,5.57782649994)
            print("lat:" + str(lat) + "  lon:" + str(lon))

            # 关闭前清除标记
            rospy.on_shutdown(cleanPath)

            rospy.spin()
        else:
            rospy.logerr("输入参数非法")
    except KeyboardInterrupt:
        print("Shutting down point_marker")