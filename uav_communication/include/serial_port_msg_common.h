#ifndef SERIAL_PORT_MSG_COMMON_H
#define SERIAL_PORT_MSG_COMMON_H

#pragma pack(1)
struct MsgHead {
    UINT8 identity1;
    UINT8 identity2;
    UINT32 len;         //长度从len后面开始算的
    //crc16从这里开始计算
    UINT8 direction;    
    UINT8 type;
    UINT8 data[0];
};
#pragma pack()

#define REV_MSG_MAX_LEN 1024                    //接收消息buffer的最大值
#define REV_MSG_HEAD_LEN 6                      //头长度
#define SND_MSG_HEAD_LEN REV_MSG_HEAD_LEN
#define MSG_TYPE_POS 7                          //类型的字节下标
#define MSG_HEAD_STRUCT_LEN (sizeof(struct MsgHead))
#define MSG_DATA_START_LEN 2
#define MSG_IDENTITY1 0xfe
#define MSG_IDENTITY2 0xff


//消息的方向
enum MESSAGE_DIRECTION_ENUM {
    FLIGHT_TO_GROUND_STATION = 0x01,
    GROUND_STATION_TO_FLIGHT = 0x02
};

//消息的类型，注意按数字大小放置
enum MESSAGE_ANALYSIS_ENUM {
    MESSAGE_ANALYSIS_FLIGHT_CONTROL                 = 0x10, //起飞、降落、gohome
    MESSAGE_ANALYSIS_ACTIVATE_WAYPOINT_MISSION      = 0x20,
    MESSAGE_ANALYSIS_SEND_LANDING_START             = 0x36, //高精度降落开始
    MESSAGE_ANALYSIS_SEND_LANDING_INFO              = 0x37,  //高精度降落发送消息
    MESSAGE_ANALYSIS_SET_ALTITUDE                   = 0x40, //设置飞机高度
    MESSAGE_ANALYSIS_HEART_BEAT                     = 0x48, //心跳
    MESSAGE_ANALYSIS_RECEIVE_STATUS                 = 0x50, //飞机发来的状态
    MESSAGE_ANALYSIS_RECEIVE_LOG_INFO               = 0x51, //飞机发来的日志信息
    MESSAGE_ANALYSIS_RECV_LANDING_INFO              = 0x55,  //高精度降落接收消息
    MESSAGE_ANALYSIS_ACTIVATE_LIDAR_NAV_MISSION     = 0x61,
    MESSAGE_ANALYSIS_PAUSE_LIDAR_NAV_MISSION        = 0x62,
    MESSAGE_ANALYSIS_RESUME_LIDAR_NAV_MISSION       = 0x63,
    MESSAGE_ANALYSIS_CANCEL_LIDAR_NAV_MISSION       = 0x64,
    MESSAGE_ANALYSIS_RESET_LIDAR_NAV_MAX_VEL        = 0x65,
    MESSAGE_ANALYSIS_SET_LIDAR_NAV_GOAL             = 0x66, //添加向无人机发送目标点的命令
    MESSAGE_ANALYSIS_CHANGE_LIDAR_NAV_FLIGHT_MODE   = 0x67,
    MESSAGE_ANALYSIS_RECEIVE_GPS_POS                = 0x70, //飞机当前GPS
    MESSAGE_ANALYSIS_RECEIVE_GPS_HEALTH             = 0x71,
    MESSAGE_ANALYSIS_RECEIVE_UAV_VELOCITY           = 0x72,
    MESSAGE_ANALYSIS_RECEIVE_HEIGHT_ABOVE_TAKEOFF   = 0x73,
    MESSAGE_ANALYSIS_RECEIVE_UAV_LOCAL_POS          = 0x74,
    MESSAGE_ANALYSIS_RECEIVE_LIDAR_NAV_REL_POS      = 0x75,
    MESSAGE_ANALYSIS_RECEIVE_LIDAR_NAV_COMPUTED_VEL = 0x76,
    MESSAGE_ANALYSIS_RECEIVE_LIDAR_NAV_ODOM         = 0x77,
    MESSAGE_ANALYSIS_SHUTDOWN_MANIFOLD              = 0x80, //关闭妙算
    MESSAGE_ANALYSIS_START_RECORD_BAG               = 0x81, //开始录包
    MESSAGE_ANALYSIS_EXIT_RECORD_BAG                = 0x82, //结束录包    
    MESSAGE_ANALYSIS_SEND_XYZ_PATH                  = 0x91, //
    MESSAGE_ANALYSIS_SET_INIT_POSE_POINT            = 0x92, //给无人机发送初始定位点
    MESSAGE_ANALYSIS_RECEIVE_NDTPOINTS              = 0x93, //接收无人机端发送的定位点
    MESSAGE_ANALYSIS_SET_STOP_MOVE_FLAG             = 0xA3, //发送停止运动
    MESSAGE_ANALYSIS_ACTIVATE_SUCCESS               = 0xA4,  //接收无人机端发送过来的激活成功标志
};

//控制消息的类型
enum MESSAGE_FLIGHT_CONTROL_ENUM {
    MESSAGE_FLIGHT_CONTROL_TAKE_OFF         = 0xA0,
    MESSAGE_FLIGHT_CONTROL_LAND             = 0xA1,
    MESSAGE_FLIGHT_CONTROL_GO_HOME          = 0xA2,
};


#endif
