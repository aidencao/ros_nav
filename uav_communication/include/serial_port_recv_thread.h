#ifndef SERIAL_PORT_RECV_H
#define SERIAL_PORT_RECV_H


#include "uavtype.h"
#include "uav_inc.h"
#include "serial_port_msg_common.h"
#include <string>
#include <geometry_msgs/Vector3.h>

class SerialPortRecv
{
public:
    static void* thread_loop(void *arg);
    static void init(void);     
private:
    SerialPortRecv();
    ~SerialPortRecv();
    static void PubDroneLogInfo(const MsgHead *pMsg);
    static void PubDroneStatusInfo(const MsgHead *pMsg);
    static void PubDroneGpsHealth(const MsgHead *pMsg);
    static void PubDroneGpsPos(const MsgHead *pMsg);
    static void PubDroneVelocity(const MsgHead *pMsg);
    static void PubDroneHeightAboveTakeoff(const MsgHead *pMsg);
    static void PubDroneUavLocalPosition(const MsgHead *pMsg);
    static void PubDroneLidarNavRelativePosition(const MsgHead *pMsg);
    static void PubDroneLidarNavComputedVelocity(const MsgHead *pMsg);
    static void PubDroneLidarNavOdom(const MsgHead *pMsg); 
    static void PubLandingInfo(const MsgHead *pMsg);  
    static void MessageAnalysis(const MsgHead *pMsg);           //分析一帧
    static void RecvMsgFromSerialAndAnalyze(void);              //获取完整一帧
};


#endif // SERIAL_PORT_SEND_RECV_H
