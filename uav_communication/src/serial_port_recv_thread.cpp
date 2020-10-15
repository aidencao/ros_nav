#include "serial_port_recv_thread.h"
#include "utility.h"
#include "crc16.h"
#include "heart_beat.h"
#include "serial_port_driver.h"
#include <ros/ros.h>
#include "ros_topic_send_recv.h"
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>


void SerialPortRecv::PubDroneLogInfo(const MsgHead *pMsg)
{
    ROS_INFO("GCS recv msg analysis : receive log info.");
    const int strlen = pMsg->len - 4;//字符串真实长度
    string logStr(&pMsg->data[0], &pMsg->data[strlen]);
    RosTopicSendRecv::pub_uav_log_info(logStr);
}

void SerialPortRecv::PubDroneGpsPos(const MsgHead *pMsg)
{
    ROS_INFO("in SerialPortRecv::PubDroneGpsPos");
    sensor_msgs::NavSatFix &refVal = *((sensor_msgs::NavSatFix *)pMsg->data);
    sensor_msgs::NavSatFix gps_pos;
    gps_pos.header.frame_id = "gps_map";//可以不要吗
    gps_pos.latitude = refVal.latitude;
    gps_pos.longitude = refVal.longitude;
    gps_pos.altitude = refVal.altitude;
    ROS_INFO("get_drone_gps---GCS recv msg analysis : \n"
        "receive drone points: latitude : %f, longitude : %f, altitude : %f", 
        gps_pos.latitude, gps_pos.longitude, gps_pos.altitude);    
    RosTopicSendRecv::pub_uav_gps_info(gps_pos);
    RosTopicSendRecv::pub_current_gps_pose(gps_pos);        
}

void SerialPortRecv::PubDroneGpsHealth(const MsgHead *pMsg)
{
    ROS_INFO("GCS recv msg analysis : receive gps health.");
    std_msgs::UInt8 gps_pos = *((std_msgs::UInt8 *)pMsg->data);
    RosTopicSendRecv::pub_uav_gps_health(gps_pos); 
}

void SerialPortRecv::PubDroneVelocity(const MsgHead *pMsg)
{
    geometry_msgs::Vector3Stamped uav_vel;
    uav_vel.vector = ((geometry_msgs::Vector3Stamped *)pMsg->data)->vector;    
    RosTopicSendRecv::pub_uav_velocity_info(uav_vel);
}

void SerialPortRecv::PubDroneHeightAboveTakeoff(const MsgHead *pMsg)
{
    std_msgs::Float32 height = *((std_msgs::Float32 *)pMsg->data);
    RosTopicSendRecv::pub_height_above_takeoff(height);
}

void SerialPortRecv::PubDroneUavLocalPosition(const MsgHead *pMsg)
{
    ROS_INFO("in SerialPortRecv::PubDroneUavLocalPosition");
    //好像带有header的数据结构不能直接赋值，否则会有问题
    geometry_msgs::PointStamped local_pos;
    local_pos.point = ((geometry_msgs::PointStamped*)pMsg->data)->point;
    RosTopicSendRecv::pub_uav_local_pos(local_pos);
}

void SerialPortRecv::PubDroneLidarNavRelativePosition(const MsgHead *pMsg)
{
    geometry_msgs::PointStamped rel_pos;
    rel_pos.point = ((geometry_msgs::PointStamped *)pMsg->data)->point;
    RosTopicSendRecv::pub_lidar_nav_rel_pos(rel_pos);
}

void SerialPortRecv::PubDroneLidarNavComputedVelocity(const MsgHead *pMsg)
{
    geometry_msgs::Vector3Stamped nav_vel;
    nav_vel.vector = ((geometry_msgs::Vector3Stamped *)pMsg->data)->vector;
    RosTopicSendRecv::pub_lidar_nav_computed_vel(nav_vel);
}

void SerialPortRecv::PubDroneLidarNavOdom(const MsgHead *pMsg)
{   
    ROS_INFO("GCS recv msg analysis : receive lidar nav odom.");
    nav_msgs::Odometry uav_odom;
    uav_odom.header.frame_id = "map";//不可以不要
    uav_odom.pose = ((nav_msgs::Odometry *)pMsg->data)->pose;    
    ROS_INFO("receive DroneLidarNavOdom points: x : %f, y : %f, z : %f", 
        uav_odom.pose.pose.position.x, uav_odom.pose.pose.position.y , uav_odom.pose.pose.position.z);
    RosTopicSendRecv::pub_lidar_nav_odom(uav_odom);
}

void SerialPortRecv::PubDroneStatusInfo(const MsgHead *pMsg)
{


}

//获取一个完整帧消息后分析
void SerialPortRecv::MessageAnalysis(const MsgHead *pMsg)
{
    if (pMsg->direction != FLIGHT_TO_GROUND_STATION) {  //判断是否为飞机发来的 需要区分
        ROS_ERROR("GCS MessageAnalysis buf[0] isnot 0x01");
        return;
    }

    switch(pMsg->type)
    {
    case MESSAGE_ANALYSIS_RECEIVE_LOG_INFO:
        PubDroneLogInfo(pMsg);
        break;
    case MESSAGE_ANALYSIS_RECEIVE_GPS_POS:
        PubDroneGpsPos(pMsg);
        break;
    case MESSAGE_ANALYSIS_RECEIVE_GPS_HEALTH:
        PubDroneGpsHealth(pMsg);
        break;
    case MESSAGE_ANALYSIS_RECEIVE_UAV_VELOCITY:
        PubDroneVelocity(pMsg);
        break;
    case MESSAGE_ANALYSIS_RECEIVE_HEIGHT_ABOVE_TAKEOFF:
        PubDroneHeightAboveTakeoff(pMsg);
        break;
    case MESSAGE_ANALYSIS_RECEIVE_UAV_LOCAL_POS:
        PubDroneUavLocalPosition(pMsg);
        break;
    case MESSAGE_ANALYSIS_RECEIVE_LIDAR_NAV_REL_POS:
        PubDroneLidarNavRelativePosition(pMsg);
        break;
    case MESSAGE_ANALYSIS_RECEIVE_LIDAR_NAV_COMPUTED_VEL:
        PubDroneLidarNavComputedVelocity(pMsg);
        break;
    case MESSAGE_ANALYSIS_RECEIVE_LIDAR_NAV_ODOM:        
        PubDroneLidarNavOdom(pMsg);
        break; 
    case MESSAGE_ANALYSIS_ACTIVATE_SUCCESS:
        {
        ROS_INFO("MESSAGE_ANALYSIS_ACTIVATE_SUCCESS----------------------received");
        ROS_INFO("MESSAGE_ANALYSIS_ACTIVATE_SUCCESS----------------------received");
        ROS_INFO("MESSAGE_ANALYSIS_ACTIVATE_SUCCESS----------------------received");
        ROS_INFO("MESSAGE_ANALYSIS_ACTIVATE_SUCCESS----------------------received");
        ROS_INFO("MESSAGE_ANALYSIS_ACTIVATE_SUCCESS----------------------received");
        ROS_INFO("MESSAGE_ANALYSIS_ACTIVATE_SUCCESS----------------------received");
        ROS_INFO("MESSAGE_ANALYSIS_ACTIVATE_SUCCESS----------------------received");
        ROS_INFO("MESSAGE_ANALYSIS_ACTIVATE_SUCCESS----------------------received");
        break;
        }
    case MESSAGE_ANALYSIS_RECEIVE_STATUS:
        ROS_INFO("GCS recv msg analysis : receive status info.");
        //ShowDroneStatusInfo(pMsg);
        break;
    default:
        //ROS_INFO("GCS recv msg analysis : unknow cmd - %x", cmd);
        break;
    }
}

//尝试获得一个完整帧的消息
void SerialPortRecv::RecvMsgFromSerialAndAnalyze()
{
    char rev_msg_buf[REV_MSG_MAX_LEN];       //得到一帧完整的数据
    int rev_idx=0;                           //index
    int rev_buf_ok = false;                  //为false表示消息还没成帧

    int ret;
    size_t nread, offset;
    UINT32 len;             //规定了4个字节所以必须为32位

    //得到第一个字符
    ret = Alex_SerialPort_Recv(&rev_msg_buf[rev_idx], 1);
    if (ret == 0) { //获取第一个字节超时就不打印了，否则打印的东西太多了
        return;
    } else if (ret < 0 || 0xfe != (UINT8)rev_msg_buf[rev_idx]) {
        ROS_INFO("msg recv : first char - %02x .", (UINT8)rev_msg_buf[rev_idx]);
        ROS_INFO("msg recv : get first char error");
        return;
    }
    rev_idx++;


    //得到第二个字符
    ret = Alex_SerialPort_Recv(&rev_msg_buf[rev_idx], 1);
    if (ret <= 0 || 0xff != (UINT8)rev_msg_buf[rev_idx]) {
        ROS_INFO("msg recv : second char - %02x .", (UINT8)rev_msg_buf[rev_idx]);
        ROS_INFO("msg recv : get second char error");
        rev_idx = 0;
        return;
    }
    rev_idx++;


    //得到长度
    nread = sizeof(len);
    offset = 0;
    while (nread > 0) {
        ret = Alex_SerialPort_Recv((char *)&len + offset, nread);
        if (ret <= 0) {
            ROS_ERROR("msg recv : get msg len error");
            rev_idx=0;
            break;
        }
        nread -= ret;
        offset += ret;
    }
    if (rev_idx == 0) return;
    //ROS_INFO("msg recv : get len - %u ", len);
    if (len + REV_MSG_HEAD_LEN > REV_MSG_MAX_LEN) {
        ROS_ERROR("msg recv : msg len overflow");
        rev_idx = 0;
        return;
    }
    *(UINT32 *)(&rev_msg_buf[rev_idx]) = len;
    rev_idx += sizeof(len);


    //得到数据
    nread = len;
    offset = 0;
    while (nread > 0) {
        ret = Alex_SerialPort_Recv(&rev_msg_buf[rev_idx + offset], nread);
        if (ret <= 0) {
            ROS_ERROR("msg recv : get msg data error");
            rev_idx=0;
            break;
        }
        nread -= ret;
        offset += ret;
    }
    if (rev_idx == 0) return;
    rev_idx += len;

    //得到crc16，这里不用获取了，上面一步已经得到了
    #if 0
    nread = CRC16_BYTES;
    offset = 0;
    while (nread > 0) {
        ret = Alex_SerialPort_Recv(&rev_msg_buf[rev_idx + offset], nread);
        if (ret <= 0) {
            ROS_ERROR("get msg crc error");
            rev_idx=0;
            break;
        }
        nread -= ret;
        offset += ret;
    }
    if (rev_idx == 0) return;
    rev_idx += CRC16_BYTES;
    #endif

    //crc信息校验
    UINT16 crc16_from_msg = *(UINT16*)(&rev_msg_buf[rev_idx - CRC16_BYTES]);
    UINT16 ctc16_from_calc = crc16(&rev_msg_buf[REV_MSG_HEAD_LEN], len - CRC16_BYTES);

        //打印信息
    //如果是心跳报文则少打印信息
    if (crc16_from_msg == ctc16_from_calc)
    {

         //ROS_INFO("rev msg buf: ");

         //string str = string(rev_msg_buf, rev_msg_buf+rev_idx);
         //PRINT_STRING_TO_BINARY(str);

         //ROS_INFO("crc16_from_msg=%p,  ctc16_from_calc=%p", crc16_from_msg, ctc16_from_calc);

    }
    else
    {
        ROS_INFO("rev msg buf: ");

        string str = string(rev_msg_buf, rev_msg_buf+rev_idx);
        PRINT_STRING_TO_BINARY(str);

        ROS_INFO("crc16_from_msg=0x%04x,  ctc16_from_calc=0x%04x", crc16_from_msg, ctc16_from_calc);
        ROS_WARN("crc16 is bad, send msg again!");
        rev_idx=0;
        return;
    }

    //得到完整的一帧
    rev_buf_ok = true;

    //只要收到过消息就要开始维持心跳
    bool old_heart_beat_flag = HeartBeat::turn_on_heart_beat_start_flag();
    if ( ! old_heart_beat_flag ) {
        ROS_INFO("turn_on_heart_beat_start_flag...");
    }

    //表示收到过消息
    HeartBeat::turn_on_heart_beat_come_flag();

    MessageAnalysis((MsgHead*)rev_msg_buf);


    //准备重新获得一帧
    rev_buf_ok = false;
    rev_idx=0;
}


void* SerialPortRecv::thread_loop(void* arg)
{
    while(true)
    {
        RecvMsgFromSerialAndAnalyze();
    }

}

void SerialPortRecv::init(void)
{
    
}
