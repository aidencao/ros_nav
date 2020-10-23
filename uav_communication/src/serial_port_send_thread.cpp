#include "serial_port_send_thread.h"
#include "utility.h"
#include "crc16.h"
#include "serial_port_driver.h"
#include <ros/ros.h>
#include "heart_beat.h"
#include <geometry_msgs/Pose.h>

//send buffer
queue<string> SerialPortSend::send_buffer_queue;
pthread_mutex_t SerialPortSend::send_buffer_queue_lock;

//send buff
void SerialPortSend::AddMsgToSendBuffer(const string &str)
{
    pthread_mutex_lock(&send_buffer_queue_lock);
    send_buffer_queue.push(str);
    pthread_mutex_unlock(&send_buffer_queue_lock);
}

bool SerialPortSend::is_send_buffer_queue_empty(void)
{
    pthread_mutex_lock(&send_buffer_queue_lock);
    bool isEmpty = send_buffer_queue.empty();
    pthread_mutex_unlock(&send_buffer_queue_lock);
    return isEmpty;
}

string SerialPortSend::getMsgFromSendBufferQueue(void)
{
    pthread_mutex_lock(&send_buffer_queue_lock);
    string tmpstr = send_buffer_queue.front();
    send_buffer_queue.pop();
    pthread_mutex_unlock(&send_buffer_queue_lock);
    return tmpstr;
}

void SerialPortSend::send_stop_move_cmd(const std_msgs::Bool::ConstPtr &ros_msg)
{
    const UINT32 REAL_DATA_LEN = 1;
    const UINT32 DATA_LEN = CRC16_BYTES + REAL_DATA_LEN;
    const UINT32 MSG_LEN = MSG_HEAD_STRUCT_LEN + DATA_LEN;

    char msg[MSG_LEN];
    MsgHead *pMsg = (MsgHead *)msg;
    pMsg->identity1 = MSG_IDENTITY1;
    pMsg->identity2 = MSG_IDENTITY2;
    pMsg->len = MSG_DATA_START_LEN + DATA_LEN;
    pMsg->direction = GROUND_STATION_TO_FLIGHT;
    pMsg->type = MESSAGE_ANALYSIS_SET_STOP_MOVE_FLAG;

    bool &refVal = *((bool *)pMsg->data);
    refVal = ros_msg->data;

    UINT16 &refCrc = *((UINT16 *)&pMsg->data[REAL_DATA_LEN]);
    refCrc = crc16(&msg[SND_MSG_HEAD_LEN], MSG_DATA_START_LEN + REAL_DATA_LEN);

    AddMsgToSendBuffer(string(&msg[0], &msg[MSG_LEN]));
}

//发送激活信息给无人机端
void SerialPortSend::send_waypoint_activation_cmd(const std_msgs::String::ConstPtr ros_msg)
{
    ROS_INFO("in send_waypoint_activation_cmd");
    const UINT32 REAL_DATA_LEN = 0;
    const UINT32 DATA_LEN = CRC16_BYTES + REAL_DATA_LEN;
    const UINT32 MSG_LEN = MSG_HEAD_STRUCT_LEN + DATA_LEN;

    char msg[MSG_LEN];
    MsgHead *pMsg = (MsgHead *)msg;
    pMsg->identity1 = MSG_IDENTITY1;
    pMsg->identity2 = MSG_IDENTITY2;
    pMsg->len = MSG_DATA_START_LEN + DATA_LEN;
    pMsg->direction = GROUND_STATION_TO_FLIGHT;
    pMsg->type = MESSAGE_ANALYSIS_ACTIVATE_WAYPOINT_MISSION;

    UINT16 &refCrc = *((UINT16 *)&pMsg->data[REAL_DATA_LEN]);
    refCrc = crc16(&msg[SND_MSG_HEAD_LEN], MSG_DATA_START_LEN + REAL_DATA_LEN);

    AddMsgToSendBuffer(string(&msg[0], &msg[MSG_LEN]));
}

void SerialPortSend::send_haltManifold_cmd(const std_msgs::Bool::ConstPtr &ros_msg)
{
    const UINT32 REAL_DATA_LEN = 0;
    const UINT32 DATA_LEN = CRC16_BYTES + REAL_DATA_LEN;
    const UINT32 MSG_LEN = MSG_HEAD_STRUCT_LEN + DATA_LEN;

    char msg[MSG_LEN];
    MsgHead *pMsg = (MsgHead *)msg;
    pMsg->identity1 = MSG_IDENTITY1;
    pMsg->identity2 = MSG_IDENTITY2;
    pMsg->len = MSG_DATA_START_LEN + DATA_LEN;
    pMsg->direction = GROUND_STATION_TO_FLIGHT;
    pMsg->type = MESSAGE_ANALYSIS_SHUTDOWN_MANIFOLD;

    UINT16 &refCrc = *((UINT16 *)&pMsg->data[REAL_DATA_LEN]);
    refCrc = crc16(&msg[SND_MSG_HEAD_LEN], MSG_DATA_START_LEN + REAL_DATA_LEN);

    AddMsgToSendBuffer(string(&msg[0], &msg[MSG_LEN]));
}

void SerialPortSend::send_takeoff_cmd(const std_msgs::Bool::ConstPtr &ros_msg)
{
    ROS_INFO("in SerialPortSend::send_takeoff_cmd");

    const UINT32 REAL_DATA_LEN = 1;
    const UINT32 DATA_LEN = CRC16_BYTES + REAL_DATA_LEN;
    const UINT32 MSG_LEN = MSG_HEAD_STRUCT_LEN + DATA_LEN;

    char msg[MSG_LEN];
    MsgHead *pMsg = (MsgHead *)msg;
    pMsg->identity1 = MSG_IDENTITY1;
    pMsg->identity2 = MSG_IDENTITY2;
    pMsg->len = MSG_DATA_START_LEN + DATA_LEN;
    pMsg->direction = GROUND_STATION_TO_FLIGHT;
    pMsg->type = MESSAGE_ANALYSIS_FLIGHT_CONTROL;

    UINT8 &refVal = *((UINT8 *)pMsg->data);
    refVal = MESSAGE_FLIGHT_CONTROL_TAKE_OFF;

    UINT16 &refCrc = *((UINT16 *)&pMsg->data[REAL_DATA_LEN]);
    refCrc = crc16(&msg[SND_MSG_HEAD_LEN], MSG_DATA_START_LEN + REAL_DATA_LEN);

    AddMsgToSendBuffer(string(&msg[0], &msg[MSG_LEN]));
}

void SerialPortSend::send_land_cmd(const std_msgs::Bool::ConstPtr &ros_msg)
{
    const UINT32 REAL_DATA_LEN = 1;
    const UINT32 DATA_LEN = CRC16_BYTES + REAL_DATA_LEN;
    const UINT32 MSG_LEN = MSG_HEAD_STRUCT_LEN + DATA_LEN;

    char msg[MSG_LEN];
    MsgHead *pMsg = (MsgHead *)msg;
    pMsg->identity1 = MSG_IDENTITY1;
    pMsg->identity2 = MSG_IDENTITY2;
    pMsg->len = MSG_DATA_START_LEN + DATA_LEN;
    pMsg->direction = GROUND_STATION_TO_FLIGHT;
    pMsg->type = MESSAGE_ANALYSIS_FLIGHT_CONTROL;

    UINT8 &refVal = *((UINT8 *)pMsg->data);
    refVal = MESSAGE_FLIGHT_CONTROL_LAND;

    UINT16 &refCrc = *((UINT16 *)&pMsg->data[REAL_DATA_LEN]);
    refCrc = crc16(&msg[SND_MSG_HEAD_LEN], MSG_DATA_START_LEN + REAL_DATA_LEN);

    AddMsgToSendBuffer(string(&msg[0], &msg[MSG_LEN]));
}

void SerialPortSend::send_gohome_cmd(const std_msgs::Bool::ConstPtr &ros_msg)
{
    const UINT32 REAL_DATA_LEN = 1;
    const UINT32 DATA_LEN = CRC16_BYTES + REAL_DATA_LEN;
    const UINT32 MSG_LEN = MSG_HEAD_STRUCT_LEN + DATA_LEN;

    char msg[MSG_LEN];
    MsgHead *pMsg = (MsgHead *)msg;
    pMsg->identity1 = MSG_IDENTITY1;
    pMsg->identity2 = MSG_IDENTITY2;
    pMsg->len = MSG_DATA_START_LEN + DATA_LEN;
    pMsg->direction = GROUND_STATION_TO_FLIGHT;
    pMsg->type = MESSAGE_ANALYSIS_FLIGHT_CONTROL;

    UINT8 &refVal = *((UINT8 *)pMsg->data);
    refVal = MESSAGE_FLIGHT_CONTROL_GO_HOME;

    UINT16 &refCrc = *((UINT16 *)&pMsg->data[REAL_DATA_LEN]);
    refCrc = crc16(&msg[SND_MSG_HEAD_LEN], MSG_DATA_START_LEN + REAL_DATA_LEN);

    AddMsgToSendBuffer(string(&msg[0], &msg[MSG_LEN]));
}

void SerialPortSend::send_resetHeight_cmd(const std_msgs::Float64::ConstPtr &ros_msg)
{
    const UINT32 REAL_DATA_LEN = sizeof(double);
    const UINT32 DATA_LEN = CRC16_BYTES + REAL_DATA_LEN;
    const UINT32 MSG_LEN = MSG_HEAD_STRUCT_LEN + DATA_LEN;

    char msg[MSG_LEN];
    MsgHead *pMsg = (MsgHead *)msg;
    pMsg->identity1 = MSG_IDENTITY1;
    pMsg->identity2 = MSG_IDENTITY2;
    pMsg->len = MSG_DATA_START_LEN + DATA_LEN;
    pMsg->direction = GROUND_STATION_TO_FLIGHT;
    pMsg->type = MESSAGE_ANALYSIS_SET_ALTITUDE;

    double &refVal = *((double *)pMsg->data);
    refVal = ros_msg->data;

    UINT16 &refCrc = *((UINT16 *)&pMsg->data[REAL_DATA_LEN]);
    refCrc = crc16(&msg[SND_MSG_HEAD_LEN], MSG_DATA_START_LEN + REAL_DATA_LEN);

    AddMsgToSendBuffer(string(&msg[0], &msg[MSG_LEN]));
}

void SerialPortSend::SendHeartBeatMsg(void)
{
    const UINT32 REAL_DATA_LEN = 0;
    const UINT32 DATA_LEN = CRC16_BYTES + REAL_DATA_LEN;
    const UINT32 MSG_LEN = MSG_HEAD_STRUCT_LEN + DATA_LEN;

    char msg[MSG_LEN];
    MsgHead *pMsg = (MsgHead *)msg;
    pMsg->identity1 = MSG_IDENTITY1;
    pMsg->identity2 = MSG_IDENTITY2;
    pMsg->len = MSG_DATA_START_LEN + DATA_LEN;
    pMsg->direction = GROUND_STATION_TO_FLIGHT;
    pMsg->type = MESSAGE_ANALYSIS_HEART_BEAT;

    UINT16 &refCrc = *((UINT16 *)&pMsg->data[REAL_DATA_LEN]);
    refCrc = crc16(&msg[SND_MSG_HEAD_LEN], MSG_DATA_START_LEN + REAL_DATA_LEN);

    AddMsgToSendBuffer(string(&msg[0], &msg[MSG_LEN]));
}

// 发送地图坐标路径
void SerialPortSend::send_xyz_path_cmd(const nav_msgs::Path::ConstPtr &path)
{
    const UINT32 REAL_DATA_LEN = sizeof(UINT8) + (path->poses.size() * sizeof(double) * 3);
    const UINT32 DATA_LEN = CRC16_BYTES + REAL_DATA_LEN;
    const UINT32 MSG_LEN = MSG_HEAD_STRUCT_LEN + DATA_LEN;

    char msg[MSG_LEN];
    MsgHead *pMsg = (MsgHead *)msg;
    pMsg->identity1 = MSG_IDENTITY1;
    pMsg->identity2 = MSG_IDENTITY2;
    pMsg->len = MSG_DATA_START_LEN + DATA_LEN;
    pMsg->direction = GROUND_STATION_TO_FLIGHT;
    pMsg->type = MESSAGE_ANALYSIS_SEND_XYZ_PATH;

    UINT8 &refPoseNum = *((UINT8 *)pMsg->data);
    refPoseNum = (UINT8)path->poses.size();

    for (int i = 0; i < path->poses.size(); i++)
    {
        geometry_msgs::Point *ppoint = (geometry_msgs::Point *)&pMsg->data[i * sizeof(geometry_msgs::Point) + 1];
        ppoint->x = path->poses[i].pose.position.x;
        ppoint->y = path->poses[i].pose.position.y;
        ppoint->z = path->poses[i].pose.position.z;
        ROS_INFO("x is : %lf,y is : %lf,z is : %lf", ppoint->x, ppoint->y, ppoint->z);
    }

    UINT16 &refCrc = *((UINT16 *)&pMsg->data[REAL_DATA_LEN]);
    refCrc = crc16(&msg[SND_MSG_HEAD_LEN], MSG_DATA_START_LEN + REAL_DATA_LEN);

    AddMsgToSendBuffer(string(&msg[0], &msg[MSG_LEN]));
}

// 接收到GPS_Waypoints信息
// 发送导航点
void SerialPortSend::send_waypoints_cmd(const std_msgs::String::ConstPtr ros_msg)
{
    vector<string> points = split(ros_msg->data, "$");
    int pack_length = points.size() * 12 + 1 + 2 + 4 + 2 + CRC16_BYTES;
    static char pack_bytes[512];
    size_t idx = 0;
    pack_bytes[idx++] = 0xfe;
    pack_bytes[idx++] = 0xff;
    *(UINT32 *)(&pack_bytes[idx]) = (UINT32)pack_length - 6;
    idx += sizeof(UINT32);
    pack_bytes[idx++] = GROUND_STATION_TO_FLIGHT;
    pack_bytes[idx++] = 0x30;
    pack_bytes[idx++] = (int)points.size();

    for (int i = 0; i < points.size(); i++)
    {
        string point = points[i];
        vector<string> gps_info = split(point, "@");
        UINT32 lat = stof(gps_info[0]) * 1e6;
        UINT32 lon = stof(gps_info[1]) * 1e6;
        int alt = stof(gps_info[2]) * 1e6;
        //double gps_ilat_convert,gps_ilon_convert;
        //不要使用坐标转换，windows地面站可能会使用
        //问题出在这里
        //Gps84_To_bd09(lat/1e6,lon/1e6,gps_ilat_convert,gps_ilon_convert);
        *(UINT32 *)(&pack_bytes[idx]) = lat;
        idx += sizeof(UINT32);
        *(UINT32 *)(&pack_bytes[idx]) = lon;
        idx += sizeof(UINT32);
        *(INT32 *)(&pack_bytes[idx]) = alt;
        idx += sizeof(INT32);

        ROS_INFO("lat is : %d,lon is : %d,alt is : %d", lat, lon, alt);
    }

    *(UINT16 *)(&pack_bytes[idx]) = (UINT16)crc16(&pack_bytes[SND_MSG_HEAD_LEN], idx - SND_MSG_HEAD_LEN); //crc16
    idx += CRC16_BYTES;
    AddMsgToSendBuffer(string(&pack_bytes[0], &pack_bytes[idx]));
}

//发送初始点定位点给无人机端
void SerialPortSend::send_lidar_nav_init_pose_point_cmd(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &init_pose)
{
    ROS_INFO("in send_lidar_nav_init_pose_point_cmd");
    const UINT32 REAL_DATA_LEN = sizeof(geometry_msgs::PoseWithCovarianceStamped);
    const UINT32 DATA_LEN = CRC16_BYTES + REAL_DATA_LEN;
    const UINT32 MSG_LEN = MSG_HEAD_STRUCT_LEN + DATA_LEN;

    char msg[MSG_LEN];
    MsgHead *pMsg = (MsgHead *)msg;
    pMsg->identity1 = MSG_IDENTITY1;
    pMsg->identity2 = MSG_IDENTITY2;
    pMsg->len = MSG_DATA_START_LEN + DATA_LEN;
    pMsg->direction = GROUND_STATION_TO_FLIGHT;
    pMsg->type = MESSAGE_ANALYSIS_SET_INIT_POSE_POINT;

    //这个指针不能直接用，要先取值在取地址
    memcpy(pMsg->data, &(*init_pose), sizeof(geometry_msgs::PoseWithCovarianceStamped));

    UINT16 &refCrc = *((UINT16 *)&pMsg->data[REAL_DATA_LEN]);
    refCrc = crc16(&msg[SND_MSG_HEAD_LEN], MSG_DATA_START_LEN + REAL_DATA_LEN);

    AddMsgToSendBuffer(string(&msg[0], &msg[MSG_LEN]));
}

void SerialPortSend::send_lidar_nav_goal_cmd(const geometry_msgs::PoseStamped::ConstPtr &nav_goal)
{
    ROS_INFO("in send_lidar_nav_goal_cmd");
    const UINT32 REAL_DATA_LEN = sizeof(geometry_msgs::PoseStamped);
    const UINT32 DATA_LEN = CRC16_BYTES + REAL_DATA_LEN;
    const UINT32 MSG_LEN = MSG_HEAD_STRUCT_LEN + DATA_LEN;

    char msg[MSG_LEN];
    MsgHead *pMsg = (MsgHead *)msg;
    pMsg->identity1 = MSG_IDENTITY1;
    pMsg->identity2 = MSG_IDENTITY2;
    pMsg->len = MSG_DATA_START_LEN + DATA_LEN;
    pMsg->direction = GROUND_STATION_TO_FLIGHT;
    pMsg->type = MESSAGE_ANALYSIS_SET_LIDAR_NAV_GOAL;

    //这个指针不能直接用，要先取值在取地址
    memcpy(pMsg->data, &(*nav_goal), sizeof(geometry_msgs::PoseStamped));

    UINT16 &refCrc = *((UINT16 *)&pMsg->data[REAL_DATA_LEN]);
    refCrc = crc16(&msg[SND_MSG_HEAD_LEN], MSG_DATA_START_LEN + REAL_DATA_LEN);

    AddMsgToSendBuffer(string(&msg[0], &msg[MSG_LEN]));
}

void SerialPortSend::send_lidar_nav_activate_cmd(const std_msgs::Bool::ConstPtr &ros_msg)
{
    ROS_INFO("in SerialPortSend::send_lidar_nav_activate_cmd");

    const UINT32 REAL_DATA_LEN = 0;
    const UINT32 DATA_LEN = CRC16_BYTES + REAL_DATA_LEN;
    const UINT32 MSG_LEN = MSG_HEAD_STRUCT_LEN + DATA_LEN;

    char msg[MSG_LEN];
    MsgHead *pMsg = (MsgHead *)msg;
    pMsg->identity1 = MSG_IDENTITY1;
    pMsg->identity2 = MSG_IDENTITY2;
    pMsg->len = MSG_DATA_START_LEN + DATA_LEN;
    pMsg->direction = GROUND_STATION_TO_FLIGHT;
    pMsg->type = MESSAGE_ANALYSIS_ACTIVATE_LIDAR_NAV_MISSION;

    UINT16 &refCrc = *((UINT16 *)&pMsg->data[REAL_DATA_LEN]);
    refCrc = crc16(&msg[SND_MSG_HEAD_LEN], MSG_DATA_START_LEN + REAL_DATA_LEN);

    AddMsgToSendBuffer(string(&msg[0], &msg[MSG_LEN]));
}

void SerialPortSend::send_lidar_nav_pause_cmd(const std_msgs::Bool::ConstPtr &ros_msg)
{
    ROS_INFO("in SerialPortSend::send_lidar_nav_pause_cmd");

    const UINT32 REAL_DATA_LEN = 0;
    const UINT32 DATA_LEN = CRC16_BYTES + REAL_DATA_LEN;
    const UINT32 MSG_LEN = MSG_HEAD_STRUCT_LEN + DATA_LEN;

    char msg[MSG_LEN];
    MsgHead *pMsg = (MsgHead *)msg;
    pMsg->identity1 = MSG_IDENTITY1;
    pMsg->identity2 = MSG_IDENTITY2;
    pMsg->len = MSG_DATA_START_LEN + DATA_LEN;
    pMsg->direction = GROUND_STATION_TO_FLIGHT;
    pMsg->type = MESSAGE_ANALYSIS_PAUSE_LIDAR_NAV_MISSION;

    UINT16 &refCrc = *((UINT16 *)&pMsg->data[REAL_DATA_LEN]);
    refCrc = crc16(&msg[SND_MSG_HEAD_LEN], MSG_DATA_START_LEN + REAL_DATA_LEN);

    AddMsgToSendBuffer(string(&msg[0], &msg[MSG_LEN]));
}

void SerialPortSend::send_lidar_nav_resume_cmd(const std_msgs::Bool::ConstPtr &ros_msg)
{
    ROS_INFO("in SerialPortSend::send_lidar_nav_resume_cmd");

    const UINT32 REAL_DATA_LEN = 0;
    const UINT32 DATA_LEN = CRC16_BYTES + REAL_DATA_LEN;
    const UINT32 MSG_LEN = MSG_HEAD_STRUCT_LEN + DATA_LEN;

    char msg[MSG_LEN];
    MsgHead *pMsg = (MsgHead *)msg;
    pMsg->identity1 = MSG_IDENTITY1;
    pMsg->identity2 = MSG_IDENTITY2;
    pMsg->len = MSG_DATA_START_LEN + DATA_LEN;
    pMsg->direction = GROUND_STATION_TO_FLIGHT;
    pMsg->type = MESSAGE_ANALYSIS_RESUME_LIDAR_NAV_MISSION;

    UINT16 &refCrc = *((UINT16 *)&pMsg->data[REAL_DATA_LEN]);
    refCrc = crc16(&msg[SND_MSG_HEAD_LEN], MSG_DATA_START_LEN + REAL_DATA_LEN);

    AddMsgToSendBuffer(string(&msg[0], &msg[MSG_LEN]));
}

void SerialPortSend::send_lidar_nav_cancel_cmd(const std_msgs::Bool::ConstPtr &ros_msg)
{
    ROS_INFO("in SerialPortSend::send_lidar_nav_cancel_cmd");

    const UINT32 REAL_DATA_LEN = 0;
    const UINT32 DATA_LEN = CRC16_BYTES + REAL_DATA_LEN;
    const UINT32 MSG_LEN = MSG_HEAD_STRUCT_LEN + DATA_LEN;

    char msg[MSG_LEN];
    MsgHead *pMsg = (MsgHead *)msg;
    pMsg->identity1 = MSG_IDENTITY1;
    pMsg->identity2 = MSG_IDENTITY2;
    pMsg->len = MSG_DATA_START_LEN + DATA_LEN;
    pMsg->direction = GROUND_STATION_TO_FLIGHT;
    pMsg->type = MESSAGE_ANALYSIS_CANCEL_LIDAR_NAV_MISSION;

    UINT16 &refCrc = *((UINT16 *)&pMsg->data[REAL_DATA_LEN]);
    refCrc = crc16(&msg[SND_MSG_HEAD_LEN], MSG_DATA_START_LEN + REAL_DATA_LEN);

    AddMsgToSendBuffer(string(&msg[0], &msg[MSG_LEN]));
}

void SerialPortSend::send_lidar_nav_reset_max_vel_cmd(const std_msgs::Float64::ConstPtr &ros_msg)
{
    const UINT32 REAL_DATA_LEN = sizeof(double);
    const UINT32 DATA_LEN = CRC16_BYTES + REAL_DATA_LEN;
    const UINT32 MSG_LEN = MSG_HEAD_STRUCT_LEN + DATA_LEN;

    char msg[MSG_LEN];
    MsgHead *pMsg = (MsgHead *)msg;
    pMsg->identity1 = MSG_IDENTITY1;
    pMsg->identity2 = MSG_IDENTITY2;
    pMsg->len = MSG_DATA_START_LEN + DATA_LEN;
    pMsg->direction = GROUND_STATION_TO_FLIGHT;
    pMsg->type = MESSAGE_ANALYSIS_RESET_LIDAR_NAV_MAX_VEL;

    double &refVal = *((double *)pMsg->data);
    refVal = ros_msg->data;

    UINT16 &refCrc = *((UINT16 *)&pMsg->data[REAL_DATA_LEN]);
    refCrc = crc16(&msg[SND_MSG_HEAD_LEN], MSG_DATA_START_LEN + REAL_DATA_LEN);

    AddMsgToSendBuffer(string(&msg[0], &msg[MSG_LEN]));
}

void SerialPortSend::send_lidar_nav_change_mode_cmd(const std_msgs::UInt8::ConstPtr &ros_msg)
{
    const UINT32 REAL_DATA_LEN = sizeof(UINT8);
    const UINT32 DATA_LEN = CRC16_BYTES + REAL_DATA_LEN;
    const UINT32 MSG_LEN = MSG_HEAD_STRUCT_LEN + DATA_LEN;

    char msg[MSG_LEN];
    MsgHead *pMsg = (MsgHead *)msg;
    pMsg->identity1 = MSG_IDENTITY1;
    pMsg->identity2 = MSG_IDENTITY2;
    pMsg->len = MSG_DATA_START_LEN + DATA_LEN;
    pMsg->direction = GROUND_STATION_TO_FLIGHT;
    pMsg->type = MESSAGE_ANALYSIS_CHANGE_LIDAR_NAV_FLIGHT_MODE;

    UINT8 &refVal = *((UINT8 *)pMsg->data);
    refVal = ros_msg->data;

    UINT16 &refCrc = *((UINT16 *)&pMsg->data[REAL_DATA_LEN]);
    refCrc = crc16(&msg[SND_MSG_HEAD_LEN], MSG_DATA_START_LEN + REAL_DATA_LEN);

    AddMsgToSendBuffer(string(&msg[0], &msg[MSG_LEN]));
}

void SerialPortSend::send_landing_info_cmd(const std_msgs::String::ConstPtr &ros_msg)
{
    const UINT32 REAL_DATA_LEN = sizeof(INT32) * 5;
    const UINT32 DATA_LEN = CRC16_BYTES + REAL_DATA_LEN;
    const UINT32 MSG_LEN = MSG_HEAD_STRUCT_LEN + DATA_LEN;

    char msg[MSG_LEN];
    MsgHead *pMsg = (MsgHead *)msg;
    pMsg->identity1 = MSG_IDENTITY1;
    pMsg->identity2 = MSG_IDENTITY2;
    pMsg->len = MSG_DATA_START_LEN + DATA_LEN;
    pMsg->direction = GROUND_STATION_TO_FLIGHT;
    pMsg->type = MESSAGE_ANALYSIS_SEND_LANDING_INFO;

    vector<string> recvData = split(ros_msg->data, " ");
    float isDectected77 = stof(recvData[0]);
    float isDectected78 = stof(recvData[5]);
    float isDectected, xCmd, yCmd, zCmd, yawCmd;

    isDectected = isDectected77;
    xCmd = stof(recvData[1]);
    yCmd = stof(recvData[2]);
    zCmd = stof(recvData[3]);
    yawCmd = stof(recvData[4]);

    if (isDectected78 == 1)
    {
        isDectected = isDectected78;
        xCmd = stof(recvData[6]);
        yCmd = stof(recvData[7]);
        zCmd = stof(recvData[8]);
        yawCmd = stof(recvData[9]);
    }

    INT32 &dcur = *((INT32 *)&pMsg->data[0 * sizeof(INT32)]);
    dcur = (INT32)(isDectected * 1e6);
    INT32 &xcur = *((INT32 *)&pMsg->data[1 * sizeof(INT32)]);
    xcur = (INT32)(xCmd * 1e6);
    INT32 &ycur = *((INT32 *)&pMsg->data[2 * sizeof(INT32)]);
    ycur = (INT32)(yCmd * 1e6);
    INT32 &zcur = *((INT32 *)&pMsg->data[3 * sizeof(INT32)]);
    zcur = (INT32)(zCmd * 1e6);
    INT32 &yawcur = *((INT32 *)&pMsg->data[4 * sizeof(INT32)]);
    yawcur = (INT32)(yawCmd * 1e6);
    ROS_INFO("xVel is : %f,yVel is : %f,yawCmd is : %f,high is : %f", xCmd, yCmd, yawCmd, zCmd);

    UINT16 &refCrc = *((UINT16 *)&pMsg->data[REAL_DATA_LEN]);
    refCrc = crc16(&msg[SND_MSG_HEAD_LEN], MSG_DATA_START_LEN + REAL_DATA_LEN);

    AddMsgToSendBuffer(string(&msg[0], &msg[MSG_LEN]));
}

void SerialPortSend::send_landing_start_cmd(const std_msgs::Bool::ConstPtr &ros_msg)
{
    ROS_INFO("in SerialPortSend::send_landing_start_cmd");

    const UINT32 REAL_DATA_LEN = 0;
    const UINT32 DATA_LEN = CRC16_BYTES + REAL_DATA_LEN;
    const UINT32 MSG_LEN = MSG_HEAD_STRUCT_LEN + DATA_LEN;

    char msg[MSG_LEN];
    MsgHead *pMsg = (MsgHead *)msg;
    pMsg->identity1 = MSG_IDENTITY1;
    pMsg->identity2 = MSG_IDENTITY2;
    pMsg->len = MSG_DATA_START_LEN + DATA_LEN;
    pMsg->direction = GROUND_STATION_TO_FLIGHT;
    pMsg->type = MESSAGE_ANALYSIS_SEND_LANDING_START;

    UINT16 &refCrc = *((UINT16 *)&pMsg->data[REAL_DATA_LEN]);
    refCrc = crc16(&msg[SND_MSG_HEAD_LEN], MSG_DATA_START_LEN + REAL_DATA_LEN);

    AddMsgToSendBuffer(string(&msg[0], &msg[MSG_LEN]));
}

void SerialPortSend::send_record_start_info_cmd(const std_msgs::Bool::ConstPtr &ros_msg)
{
    ROS_INFO("in SerialPortSend::send_record_start_info_cmd");
    const UINT32 REAL_DATA_LEN = 0;
    const UINT32 DATA_LEN = CRC16_BYTES + REAL_DATA_LEN;
    const UINT32 MSG_LEN = MSG_HEAD_STRUCT_LEN + DATA_LEN;

    char msg[MSG_LEN];
    MsgHead *pMsg = (MsgHead *)msg;
    pMsg->identity1 = MSG_IDENTITY1;
    pMsg->identity2 = MSG_IDENTITY2;
    pMsg->len = MSG_DATA_START_LEN + DATA_LEN;
    pMsg->direction = GROUND_STATION_TO_FLIGHT;
    pMsg->type = MESSAGE_ANALYSIS_START_RECORD_BAG;

    UINT16 &refCrc = *((UINT16 *)&pMsg->data[REAL_DATA_LEN]);
    refCrc = crc16(&msg[SND_MSG_HEAD_LEN], MSG_DATA_START_LEN + REAL_DATA_LEN);

    AddMsgToSendBuffer(string(&msg[0], &msg[MSG_LEN]));
}

void SerialPortSend::send_record_end_info_cmd(const std_msgs::Bool::ConstPtr &ros_msg)
{
    ROS_INFO("in SerialPortSend::send_record_end_info_cmd");
    const UINT32 REAL_DATA_LEN = 0;
    const UINT32 DATA_LEN = CRC16_BYTES + REAL_DATA_LEN;
    const UINT32 MSG_LEN = MSG_HEAD_STRUCT_LEN + DATA_LEN;

    char msg[MSG_LEN];
    MsgHead *pMsg = (MsgHead *)msg;
    pMsg->identity1 = MSG_IDENTITY1;
    pMsg->identity2 = MSG_IDENTITY2;
    pMsg->len = MSG_DATA_START_LEN + DATA_LEN;
    pMsg->direction = GROUND_STATION_TO_FLIGHT;
    pMsg->type = MESSAGE_ANALYSIS_EXIT_RECORD_BAG;

    UINT16 &refCrc = *((UINT16 *)&pMsg->data[REAL_DATA_LEN]);
    refCrc = crc16(&msg[SND_MSG_HEAD_LEN], MSG_DATA_START_LEN + REAL_DATA_LEN);

    AddMsgToSendBuffer(string(&msg[0], &msg[MSG_LEN]));
}

//向串口发送数据到buffer为空为止
void SerialPortSend::SendBufferQueueToSerialPort()
{
    while (!is_send_buffer_queue_empty())
    {
        pthread_mutex_lock(&send_buffer_queue_lock);
        string tmpstr = send_buffer_queue.front();
        send_buffer_queue.pop();
        pthread_mutex_unlock(&send_buffer_queue_lock);

        if ((UINT8)tmpstr[MSG_TYPE_POS] == (UINT8)MESSAGE_ANALYSIS_HEART_BEAT)
        {
            //心跳信息就不打印了，否则日志太多了
            ROS_INFO("send heart beat msg.");
        }
        else
        {
            ROS_INFO("send msg_buf: ");
            PRINT_STRING_TO_BINARY(tmpstr);
        }

        Alex_SerialPort_Send(tmpstr.c_str(), tmpstr.size());
    }
}

void *SerialPortSend::thread_loop(void *arg)
{
    while (true)
    {
        SendBufferQueueToSerialPort();
    }
}

void SerialPortSend::init(void)
{
    //初始化锁
    pthread_mutex_init(&send_buffer_queue_lock, NULL);
}
