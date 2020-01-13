/**
 地面站端
 */


#include "server.h"
#include "sendAndReceiveCenter.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>

geometry_msgs::PoseStamped localizer_pose;  // pose of sensor
pthread_mutex_t initial_position_lock;
pthread_mutex_t basic_cmd_lock;
pthread_mutex_t drone_height_lock;
ros::Time current_time;
ros::Publisher current_pose_pub;

static void localizer_pose_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  localizer_pose = *msg;
}

static void initialpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  ROS_INFO("initial pose received!");
  pthread_mutex_lock(&initial_position_lock);
  const auto& p = msg->pose.pose.position;
  const auto& q = msg->pose.pose.orientation;
  std::vector<double> pose(7);
  pose[0] = p.x;
  pose[1] = p.y;
  pose[2] = p.z;
  pose[3] = q.w;
  pose[4] = q.x;
  pose[5] = q.y;
  pose[6] = q.z;
  //发布到消息队列里发送
  pthread_mutex_unlock(&initial_position_lock);
}

static void basic_cmd_callback(const std_msgs::Int8::ConstPtr& msg)
{
  ROS_INFO("basic_cmd received!");
  pthread_mutex_lock(&basic_cmd_lock);
  send_control_cmd(msg->data);
  pthread_mutex_unlock(&basic_cmd_lock);
  
}

static void drone_height_cmd_callback(const std_msgs::Float32::ConstPtr& msg)
{
  ROS_INFO("drone_height_cmd received!");
  pthread_mutex_lock(&drone_height_lock);
  send_drone_height_cmd(msg->data);
  pthread_mutex_unlock(&drone_height_lock);
}




int main(int argc, char* argv[])
{
  int ret;
  
  ros::init(argc, argv, "communication");
  ROS_INFO("uav_communication start");
  ros::NodeHandle nh;
  
  //获取串口的端口号和波特率
  string serial_port_name;  
  int serial_baud_rate;
  
  nh.param<string>("serial_port_name", serial_port_name, string("ttyUSB0"));
  nh.param<int>("serial_baud_rate", serial_baud_rate, 57600);
  ROS_INFO("serial_port_name:%s", serial_port_name.c_str());
  ROS_INFO("serial_baud_rate:%d", serial_baud_rate);
  
  
  current_pose_pub = nh.advertise<nav_msgs::Odometry>("/odom", 5);
  
  ros::Subscriber basic_cmd_sub = nh.subscribe("/control_cmd", 5, &basic_cmd_callback);
  ros::Subscriber drone_height_cmd_sub = nh.subscribe("/drone_height_cmd", 5, &drone_height_cmd_callback);
  //ros::Subscriber localizer_sub = nh.subscribe("/localizer_pose",1, &localizer_pose_callback);
  //ros::Subscriber initialpose_sub = nh.subscribe("/initialpose", 2, &initialpose_callback);
  //初始化锁
  pthread_mutex_init(&initial_position_lock, NULL);
  pthread_mutex_init(&basic_cmd_lock, NULL);
  pthread_mutex_init(&drone_height_lock, NULL);
  //gcs线程初始化
  ROS_INFO("GCS_InitFunc...");
  GCS_InitFunc();
  
  //串口链路初始化
  ROS_INFO("Alex_SerialPort_Setup...");
  if(Alex_SerialPort_Setup(serial_port_name.c_str(), serial_baud_rate) < 0)
  {
    ROS_ERROR("Alex_SerialPort_Setup error!");
    return ret;
  }
  
  ROS_INFO("create Ground_Station_Receive_loop thread");
  pthread_t rec_msg_buf_thread;
  ret = pthread_create(&rec_msg_buf_thread, 0, GCS_SendAndRecvLoop, NULL);
  if( ret != 0 )
  {
      ROS_ERROR("Create rec_msg_buf_thread error!");
      return ret;
   }
   ros::Rate loop_rate(1);  
   //主程序
   while(ros::ok())
   {
     ros::spinOnce();
     //test
     loop_rate.sleep();
     //接收
   }
   return 0;
}


//发布odom给octomap
void pub_localizer_pose(const vector<double> pose)
{
  current_time = ros::Time::now();
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "map";
  
  //set the position
  odom.pose.pose.position.x = pose[0];
  odom.pose.pose.position.y = pose[1];
  odom.pose.pose.position.z = pose[2];
  odom.pose.pose.orientation.w = pose[3];
  odom.pose.pose.orientation.x = pose[4];
  odom.pose.pose.orientation.y = pose[5];
  odom.pose.pose.orientation.z = pose[6];
  
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = 0.0;
  
  current_pose_pub.publish(odom);
}