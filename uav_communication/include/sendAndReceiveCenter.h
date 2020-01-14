#ifndef DEMO_SENDANDRECEIVECENTER_H
#define DEMO_SENDANDRECEIVECENTER_H

#include "uavInc.h"
#include "uavtype.h"

#define REV_MSG_HEAD_LEN 6                      //头长度
#define SND_MSG_HEAD_LEN REV_MSG_HEAD_LEN
#define TAKE_OFF 11
#define LAND 12
#define MSG_TYPE_POS 7                          //类型的字节下标

//消息的方向
enum MESSAGE_DIRECTION_ENUM {
    FLIGHT_TO_GROUND_STATION = 0x01,
    GROUND_STATION_TO_FLIGHT = 0x02
};

//消息的类型
enum MESSAGE_ANALYSIS_ENUM {
    MESSAGE_ANALYSIS_RECEIVE_NDTPOINTS              = 0x11,
    MESSAGE_ANALYSIS_FLIGHT_CONTROL                 = 0x10,
    MESSAGE_ANALYSIS_SET_ALTITUDE                   = 0x40,
    
};

enum MESSAGE_FLIGHT_CONTROL_ENUM {
    MESSAGE_FLIGHT_CONTROL_TAKE_OFF         = 0xa0,
    MESSAGE_FLIGHT_CONTROL_LAND             = 0xa1,
};

#define PRINT_STRING_TO_BINARY(str) do { \
    for (auto it=str.begin(); it!=str.end(); it++)  \
        printf("%02x ", (UINT8)(*it));   \
    printf("\n");   \
} while (0)


void *GCS_SendAndRecvLoop(void *arg);

void send_control_cmd(const int cmd);

void send_drone_height_cmd(const float cmd);

void send_waypoints_cmd(const string cmd);

void GCS_InitFunc();
#endif