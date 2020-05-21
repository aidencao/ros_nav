#!/usr/bin/env python
# coding:utf-8
import rospy
from nav_msgs.msg import Odometry

if __name__ == '__main__':
    try:
        rospy.init_node("publish_node")
        rospy.loginfo("Starting publish_node")

        gps_pub = rospy.Publisher("odom", Odometry,
                                  queue_size=1)  # 用于向串口发送GPS路径

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map"

        odom.pose.pose.position.x = 11
        odom.pose.pose.position.y = 11
        odom.pose.pose.position.z = 10

        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0

        gps_pub.publish(odom)

    except KeyboardInterrupt:
        print("Shutting down point_marker")
