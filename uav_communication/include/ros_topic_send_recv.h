#ifndef ROS_TOPIC_SEND_RECV_H
#define ROS_TOPIC_SEND_RECV_H

#include "uavtype.h"
#include "uav_inc.h"

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>

class RosTopicSendRecv
{
public:
    static void init(void);
    static void pub_uav_log_info(const string &loginfo);
    static void pub_uav_gps_info(const sensor_msgs::NavSatFix &gpsinfo);
    static void pub_current_gps_pose(const sensor_msgs::NavSatFix &pub_current_gps_pose);
    static void pub_uav_gps_health(const std_msgs::UInt8 &gps_health);
    static void pub_uav_velocity_info(const geometry_msgs::Vector3Stamped &uav_vel);
    static void pub_height_above_takeoff(const std_msgs::Float32 &height);
    static void pub_uav_local_pos(const geometry_msgs::PointStamped &local_pos);
    static void pub_lidar_nav_rel_pos(const geometry_msgs::PointStamped &rel_pos);
    static void pub_lidar_nav_computed_vel(const geometry_msgs::Vector3Stamped &uav_vel);
    static void pub_lidar_nav_odom(const nav_msgs::Odometry &uav_odom);
    static void pub_landing_info();

private:
    RosTopicSendRecv() = delete;
    ~RosTopicSendRecv() = delete;

private:
    //Publisher
    static ros::Publisher log_info_pub;
    static ros::Publisher gps_info_pub;
    static ros::Publisher current_gps_pose_pub;
    static ros::Publisher gps_health_pub;
    static ros::Publisher uav_vel_pub;
    static ros::Publisher height_above_takeoff_pub;
    static ros::Publisher uav_local_pos_pub;
    static ros::Publisher lidar_nav_rel_pos_pub;
    static ros::Publisher lidar_nav_computed_vel_pub;
    static ros::Publisher lidar_nav_odom_pub;
    static ros::Publisher landing_info_pub;
    //Subscriber
    static ros::Subscriber takeoff_sub;
    static ros::Subscriber land_sub;
    static ros::Subscriber gohome_sub;
    static ros::Subscriber halt_manifold_sub;
    static ros::Subscriber set_height_sub;
    static ros::Subscriber set_height_by_move_sub;
    static ros::Subscriber waypoints_cmd_sub;
    static ros::Subscriber xyz_cmd_sub;
    static ros::Subscriber move_base_simple_sub;
    static ros::Subscriber stop_move_sub;
    static ros::Subscriber initialpose_sub;
    static ros::Subscriber Activation_cmd_sub;
    static ros::Subscriber activate_lidar_nav_sub;
    static ros::Subscriber pause_lidar_nav_sub;
    static ros::Subscriber resume_lidar_nav_sub;
    static ros::Subscriber cancel_lidar_nav_sub;
    static ros::Subscriber reset_lidar_nav_max_vel_sub;
    static ros::Subscriber change_lidar_nav_fligt_mode_sub;
    static ros::Subscriber landing_info_sub;
    static ros::Subscriber landing_start_sub;
    static ros::Subscriber record_map_start_sub;
    static ros::Subscriber record_map_end_sub;
};

#endif
