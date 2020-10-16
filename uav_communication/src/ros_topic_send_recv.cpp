
#include "ros_topic_send_recv.h"
#include "serial_port_send_thread.h"
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

//
ros::Publisher RosTopicSendRecv::log_info_pub;
ros::Publisher RosTopicSendRecv::gps_info_pub;
ros::Publisher RosTopicSendRecv::current_gps_pose_pub;
ros::Publisher RosTopicSendRecv::gps_health_pub;
ros::Publisher RosTopicSendRecv::uav_vel_pub;
ros::Publisher RosTopicSendRecv::height_above_takeoff_pub;
ros::Publisher RosTopicSendRecv::uav_local_pos_pub;
ros::Publisher RosTopicSendRecv::lidar_nav_rel_pos_pub;
ros::Publisher RosTopicSendRecv::lidar_nav_computed_vel_pub;
ros::Publisher RosTopicSendRecv::lidar_nav_odom_pub;
ros::Publisher RosTopicSendRecv::landing_info_pub;

//
ros::Subscriber RosTopicSendRecv::takeoff_sub;
ros::Subscriber RosTopicSendRecv::land_sub;
ros::Subscriber RosTopicSendRecv::gohome_sub;
ros::Subscriber RosTopicSendRecv::halt_manifold_sub;
ros::Subscriber RosTopicSendRecv::set_height_sub;
ros::Subscriber RosTopicSendRecv::waypoints_cmd_sub;
ros::Subscriber RosTopicSendRecv::xyz_cmd_sub;
ros::Subscriber RosTopicSendRecv::move_base_simple_sub;
ros::Subscriber RosTopicSendRecv::stop_move_sub;
ros::Subscriber RosTopicSendRecv::initialpose_sub;
ros::Subscriber RosTopicSendRecv::Activation_cmd_sub;
ros::Subscriber RosTopicSendRecv::activate_lidar_nav_sub;
ros::Subscriber RosTopicSendRecv::pause_lidar_nav_sub;
ros::Subscriber RosTopicSendRecv::resume_lidar_nav_sub;
ros::Subscriber RosTopicSendRecv::cancel_lidar_nav_sub;
ros::Subscriber RosTopicSendRecv::reset_lidar_nav_max_vel_sub;
ros::Subscriber RosTopicSendRecv::change_lidar_nav_fligt_mode_sub;
ros::Subscriber RosTopicSendRecv::landing_info_sub;
ros::Subscriber RosTopicSendRecv::landing_start_sub;

//发布日志信息给Qt
void RosTopicSendRecv::pub_uav_log_info(const string &loginfo)
{
    std_msgs::String logstr;
    logstr.data = loginfo;
    log_info_pub.publish(logstr);
}

void RosTopicSendRecv::pub_uav_gps_info(const sensor_msgs::NavSatFix &gpsinfo)
{
    ROS_INFO("in RosTopicSendRecv::pub_uav_gps_info");
    gps_info_pub.publish(gpsinfo);
}

void RosTopicSendRecv::pub_current_gps_pose(const sensor_msgs::NavSatFix &gpsinfo)
{
    ROS_INFO("in RosTopicSendRecv::pub_current_gps_pose");
    current_gps_pose_pub.publish(gpsinfo);
}

void RosTopicSendRecv::pub_uav_gps_health(const std_msgs::UInt8 &gps_health)
{
    gps_health_pub.publish(gps_health);
}

void RosTopicSendRecv::pub_uav_velocity_info(const geometry_msgs::Vector3Stamped &uav_vel)
{
    uav_vel_pub.publish(uav_vel);
}

void RosTopicSendRecv::pub_height_above_takeoff(const std_msgs::Float32 &height)
{
    height_above_takeoff_pub.publish(height);
}

void RosTopicSendRecv::pub_uav_local_pos(const geometry_msgs::PointStamped &local_pos)
{
    ROS_INFO("in RosTopicSendRecv::pub_uav_local_pos");
    uav_local_pos_pub.publish(local_pos);
}

void RosTopicSendRecv::pub_lidar_nav_rel_pos(const geometry_msgs::PointStamped &rel_pos)
{
    lidar_nav_rel_pos_pub.publish(rel_pos);
}

void RosTopicSendRecv::pub_lidar_nav_computed_vel(const geometry_msgs::Vector3Stamped &uav_vel)
{
    lidar_nav_computed_vel_pub.publish(uav_vel);
}

void RosTopicSendRecv::pub_lidar_nav_odom(const nav_msgs::Odometry &uav_odom)
{
    lidar_nav_odom_pub.publish(uav_odom);
}

void RosTopicSendRecv::pub_landing_info()
{
    std_msgs::String info;
    info.data = "uav need data";
    landing_info_pub.publish(info);
}

void RosTopicSendRecv::init(void)
{
    ros::NodeHandle nh;

    //pub
    log_info_pub = nh.advertise<std_msgs::String>("/uav_log_info", 2);
    gps_info_pub = nh.advertise<sensor_msgs::NavSatFix>("/uav_gps_info", 1);
    current_gps_pose_pub = nh.advertise<sensor_msgs::NavSatFix>("/raw_fix", 1);
    gps_health_pub = nh.advertise<std_msgs::UInt8>("/uav_gps_health", 1);
    uav_vel_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/uav_velocity_info", 1);
    height_above_takeoff_pub = nh.advertise<std_msgs::Float32>("/height_above_takeoff", 1);
    uav_local_pos_pub = nh.advertise<geometry_msgs::PointStamped>("/uav_local_pos", 1);
    lidar_nav_rel_pos_pub = nh.advertise<geometry_msgs::PointStamped>("/lidar_nav_rel_pos", 1);
    lidar_nav_computed_vel_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/lidar_nav_computed_vel", 1);
    lidar_nav_odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1);
    //降落pub
    landing_info_pub = nh.advertise<std_msgs::String>("/landing_recv", 1);

    //sub
    takeoff_sub = nh.subscribe("/uav_take_off", 1, &SerialPortSend::send_takeoff_cmd);
    land_sub = nh.subscribe("/uav_land", 1, &SerialPortSend::send_land_cmd);
    gohome_sub = nh.subscribe("/uav_gohome", 1, &SerialPortSend::send_gohome_cmd);
    halt_manifold_sub = nh.subscribe("/uav_halt_manifold", 1, &SerialPortSend::send_haltManifold_cmd);
    set_height_sub = nh.subscribe<std_msgs::Float64>("/set_uav_height", 1, &SerialPortSend::send_resetHeight_cmd);
    waypoints_cmd_sub = nh.subscribe("gps/path", 1, &SerialPortSend::send_waypoints_cmd);
    xyz_cmd_sub = nh.subscribe("xyz/path", 1, &SerialPortSend::send_xyz_path_cmd);
    stop_move_sub = nh.subscribe<std_msgs::Bool>("/stop_move_flag", 1, &SerialPortSend::send_stop_move_cmd);
    //激活GPS  Activation
    Activation_cmd_sub = nh.subscribe("/Activation", 1, &SerialPortSend::send_waypoint_activation_cmd);
    //lidar nav
    initialpose_sub = nh.subscribe("/initialpose", 1, &SerialPortSend::send_lidar_nav_init_pose_point_cmd);
    move_base_simple_sub = nh.subscribe("/move_base_simple/goal", 1, &SerialPortSend::send_lidar_nav_goal_cmd);
    activate_lidar_nav_sub = nh.subscribe("/lidar_nav_activate", 1, SerialPortSend::send_lidar_nav_activate_cmd);
    pause_lidar_nav_sub = nh.subscribe("/lidar_nav_pause", 1, SerialPortSend::send_lidar_nav_pause_cmd);
    resume_lidar_nav_sub = nh.subscribe("/lidar_nav_resume", 1, SerialPortSend::send_lidar_nav_resume_cmd);
    cancel_lidar_nav_sub = nh.subscribe("/lidar_nav_cancel", 1, SerialPortSend::send_lidar_nav_cancel_cmd);
    reset_lidar_nav_max_vel_sub = nh.subscribe("/lidar_nav_set_max_vel", 1, SerialPortSend::send_lidar_nav_reset_max_vel_cmd);
    change_lidar_nav_fligt_mode_sub = nh.subscribe("/lidar_nav_change_mode", 1, SerialPortSend::send_lidar_nav_change_mode_cmd);
    //降落sub
    landing_info_sub = nh.subscribe("/landing_send", 1, SerialPortSend::send_landing_info_cmd);
    landing_start_sub = nh.subscribe("/qt_landing_start", 1, SerialPortSend::send_landing_start_cmd);
}
