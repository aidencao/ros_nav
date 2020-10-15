#ifndef SERIAL_PORT_SEND_THREAD_H
#define SERIAL_PORT_SEND_THREAD_H

#include "uavtype.h"
#include "uav_inc.h"
#include "serial_port_msg_common.h"
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>



class SerialPortSend
{
public:
    static void* thread_loop(void *arg);
    static void init(void);  
    static void SendHeartBeatMsg(void);    
    static void send_takeoff_cmd(const std_msgs::Bool::ConstPtr& ros_msg);
    static void send_land_cmd(const std_msgs::Bool::ConstPtr& ros_msg);
    static void send_resetHeight_cmd(const std_msgs::Float64::ConstPtr& ros_msg);
    static void send_stop_move_cmd(const std_msgs::Bool::ConstPtr& ros_msg);
    static void send_haltManifold_cmd(const std_msgs::Bool::ConstPtr& ros_msg);
    static void send_gohome_cmd(const std_msgs::Bool::ConstPtr& ros_msg);
    static void send_xyz_path_cmd(const nav_msgs::Path::ConstPtr &path);
    static void send_waypoints_cmd(const std_msgs::String::ConstPtr ros_msg);  
    static void send_waypoint_activation_cmd(const std_msgs::String::ConstPtr ros_msg);
    static void send_lidar_nav_goal_cmd(const geometry_msgs::PoseStamped::ConstPtr& ros_msg);
    static void send_lidar_nav_init_pose_point_cmd(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ros_msg);
    static void send_lidar_nav_activate_cmd(const std_msgs::Bool::ConstPtr& ros_msg);
    static void send_lidar_nav_pause_cmd(const std_msgs::Bool::ConstPtr& ros_msg);
    static void send_lidar_nav_resume_cmd(const std_msgs::Bool::ConstPtr& ros_msg);
    static void send_lidar_nav_cancel_cmd(const std_msgs::Bool::ConstPtr& ros_msg);
    static void send_lidar_nav_reset_max_vel_cmd(const std_msgs::Float64::ConstPtr& ros_msg);
    static void send_lidar_nav_change_mode_cmd(const std_msgs::UInt8::ConstPtr& ros_msg);
private:
    SerialPortSend();
    ~SerialPortSend();
    static void AddMsgToSendBuffer(const string &str);
    static bool is_send_buffer_queue_empty(void);
    static string getMsgFromSendBufferQueue(void);
    static void SendBufferQueueToSerialPort();
private:
    //data
    static queue<string> send_buffer_queue;
    static pthread_mutex_t send_buffer_queue_lock;
};

#endif
