/**
 地面站端
 */
#include "uavtype.h"
#include "uav_inc.h"
#include "heart_beat.h"
#include "ros_topic_send_recv.h"
#include "serial_port_driver.h"
#include "serial_port_send_thread.h"
#include "serial_port_recv_thread.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "communication");
    ROS_INFO("uav_communication start");

    //
    ROS_INFO("serial_port_init...");
    //int ret = serial_port_init();
    string serial_port_name;
    int serial_baud_rate;
    ros::NodeHandle nh;
    nh.param<string>("serial_port_name", serial_port_name, string("ttyUSB0"));
    nh.param<int>("serial_baud_rate", serial_baud_rate, 57600);
    ROS_INFO("serial_port_name:%s", serial_port_name.c_str());
    ROS_INFO("serial_baud_rate:%d", serial_baud_rate);
    //串口初始化
    ROS_INFO("Alex_SerialPort_Setup...");
    int ret = Alex_SerialPort_Setup(serial_port_name.c_str(), serial_baud_rate);
    if(ret < 0)
    {
        ROS_ERROR("Alex_SerialPort_Setup error!");
        return ret;
    }   

    //串口发送线程初始化
    ROS_INFO("SerialPortSend::init...");
    SerialPortSend::init();
    //接收串口数据的线程
    ROS_INFO("SerialPortRecv::init...");
    SerialPortRecv::init();
    //心跳线程初始化
    ROS_INFO("HeartBeat::heart_beat_init...");
    HeartBeat::init();
    ROS_INFO("RosTopicSendRecv::init...");
    RosTopicSendRecv::init();
    if( ret != 0 ){
        ROS_ERROR("Create RosTopicSendRecv::init error!");
        return ret;
    }
    
    //发送串口数据的线程
    ROS_INFO("create SerialPortSend::thread_loop thread");
    pthread_t send_thread;
    ret = pthread_create(&send_thread, 0, SerialPortSend::thread_loop, NULL);
    if( ret != 0 )
    {
        ROS_ERROR("Create SerialPortSend::thread_loop error!");
        return ret;
    }
    
    //接收发送串口数据的线程
    ROS_INFO("create SerialPortRecv::thread_loop thread");
    pthread_t recv_thread;
    ret = pthread_create(&recv_thread, 0, SerialPortRecv::thread_loop, NULL);
    if( ret != 0 )
    {
        ROS_ERROR("Create SerialPortRecv::thread_loop error!");
        return ret;
    }

    //开个线程处理心跳报文  
    ROS_INFO("create UAV_HandHeartBeatLoop thread");
    pthread_t hb_thread;
    ret = pthread_create(&hb_thread, 0, HeartBeat::thread_loop, NULL);
    if( ret != 0 ){
        ROS_ERROR("Create handle_heart_beat_thread error!");
        return ret;
    }    

    ROS_INFO("init ok, ready to ros::spin");
    ros::spin();
    //while(1){sleep(5);};
    
    return 0;
}


