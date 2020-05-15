#include "sendAndReceiveCenter.h"
#include "crc16.h"
#include "server.h"
#include "alexSerialPort.h"
#include <ros/ros.h>

queue<string> send_buffer_queue;
pthread_mutex_t send_buffer_queue_lock;

//字符串分割函数
std::vector<std::string> split(std::string str, std::string pattern)
{
    std::string::size_type pos;
    std::vector<std::string> result;
    str += pattern; //扩展字符串以方便操作
    int size = str.size();

    for (int i = 0; i < size; i++)
    {
        pos = str.find(pattern, i);
        if (pos < size)
        {
            std::string s = str.substr(i, pos - i);
            result.push_back(s);
            i = pos + pattern.size() - 1;
        }
    }
    return result;
}

//发送消息
void GCS_AddMsgToSendBuffer(const char *msg, size_t len)
{
    string tmpstr(msg, msg + len);
    pthread_mutex_lock(&send_buffer_queue_lock);
    send_buffer_queue.push(tmpstr);
    pthread_mutex_unlock(&send_buffer_queue_lock);
}

static bool is_send_buffer_queue_empty()
{
    pthread_mutex_lock(&send_buffer_queue_lock);
    bool empty = send_buffer_queue.empty();
    pthread_mutex_unlock(&send_buffer_queue_lock);
    return empty;
}

void send_control_cmd(const int cmd)
{
    static char control_msg[10];
    size_t idx = 0;
    control_msg[idx++] = 0xfe; //header
    control_msg[idx++] = 0xff;
    *(UINT32 *)(&control_msg[idx]) = (UINT32)(2 + 1 + CRC16_BYTES); //len
    idx += sizeof(UINT32);
    control_msg[idx++] = GROUND_STATION_TO_FLIGHT; //direction
    control_msg[idx++] = MESSAGE_ANALYSIS_FLIGHT_CONTROL;
    //判断是起飞还是降落
    if (cmd == TAKE_OFF)
    {
        control_msg[idx++] = MESSAGE_FLIGHT_CONTROL_TAKE_OFF;
        ROS_INFO("NOW EXECUTE TAKE OFF!");
    }
    else if (cmd == LAND)
    {
        control_msg[idx++] = MESSAGE_FLIGHT_CONTROL_LAND;
        ROS_INFO("NOW EXECUTE LAND!");
    }
    else
    {
        ROS_ERROR("GET CONTROL CMD ERROR : %d", cmd);
        return;
    }
    *(UINT16 *)(&control_msg[idx]) = (UINT16)crc16(&control_msg[SND_MSG_HEAD_LEN], idx - SND_MSG_HEAD_LEN); //crc16
    idx += CRC16_BYTES;

    GCS_AddMsgToSendBuffer(control_msg, idx);
}

void send_drone_height_cmd(const float cmd)
{
    static char drone_height_msg[10];
    size_t idx = 0;
    drone_height_msg[idx++] = 0xfe; //header
    drone_height_msg[idx++] = 0xff;
    *(UINT32 *)(&drone_height_msg[idx]) = (UINT32)(2 + 4 + CRC16_BYTES); //len
    idx += sizeof(UINT32);
    drone_height_msg[idx++] = GROUND_STATION_TO_FLIGHT; //direction
    drone_height_msg[idx++] = MESSAGE_ANALYSIS_SET_ALTITUDE;
    UINT32 height = cmd * 1e6;
    *(UINT32 *)(&drone_height_msg[idx]) = height;
    idx += sizeof(UINT32);

    *(UINT16 *)(&drone_height_msg[idx]) = (UINT16)crc16(&drone_height_msg[SND_MSG_HEAD_LEN], idx - SND_MSG_HEAD_LEN); //crc16
    idx += CRC16_BYTES;

    GCS_AddMsgToSendBuffer(drone_height_msg, idx);
}

// 发送导航点
void send_waypoints_cmd(const string cmd)
{
    vector<string> points = split(cmd, "$");
    int pack_length = points.size() * 12 + 1 + 2 + 4 + 2 + CRC16_BYTES;
    static char pack_bytes[512];
    size_t idx = 0;
    pack_bytes[idx++] = 0xfe;
    pack_bytes[idx++] = 0xff;
    *(UINT32 *)(&pack_bytes[idx]) = (UINT32)pack_length - 6;
    idx += sizeof(UINT32);
    pack_bytes[idx++] = 0x02;
    pack_bytes[idx++] = 0x30;
    pack_bytes[idx++] = (int)points.size();

    for (int i = 0; i < points.size(); i++)
    {
        string point = points[i];
        vector<string> gps_info = split(point, "@");
        UINT32 lat = stof(gps_info[0]) * 1e6;
        UINT32 lon = stof(gps_info[1]) * 1e6;
        int alt = stof(gps_info[2]) * 1e6;
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
    GCS_AddMsgToSendBuffer(pack_bytes, idx);
}

// 发送地图坐标路径
void send_xyz_path_cmd(const nav_msgs::PathConstPtr &path)
{
    int pack_length = path->poses.size() * 12 + 1 + 2 + 4 + 2 + CRC16_BYTES;
    static char pack_bytes[512];
    size_t idx = 0;
    pack_bytes[idx++] = 0xfe;
    pack_bytes[idx++] = 0xff;
    *(UINT32 *)(&pack_bytes[idx]) = (UINT32)pack_length - 6;
    idx += sizeof(UINT32);
    pack_bytes[idx++] = 0x02;
    pack_bytes[idx++] = 0x91;
    pack_bytes[idx++] = (int)path->poses.size();

    for (int i = 0; i < path->poses.size(); i++)
    {
        geometry_msgs::Point point = path->poses[i].pose.position;
        int x = (point.x) * 1e6;
        int y = (point.y) * 1e6;
        int z = (point.z) * 1e6;
        *(INT32 *)(&pack_bytes[idx]) = x;
        idx += sizeof(INT32);
        *(INT32 *)(&pack_bytes[idx]) = y;
        idx += sizeof(INT32);
        *(INT32 *)(&pack_bytes[idx]) = z;
        idx += sizeof(INT32);

        ROS_INFO("x is : %d,y is : %d,z is : %d", x, y, z);
    }

    *(UINT16 *)(&pack_bytes[idx]) = (UINT16)crc16(&pack_bytes[SND_MSG_HEAD_LEN], idx - SND_MSG_HEAD_LEN); //crc16
    idx += CRC16_BYTES;
    GCS_AddMsgToSendBuffer(pack_bytes, idx);
}

static void get_ndt_points(const char *buf)
{
    vector<double> posVector(7);
    INT32 xCmd = *(INT32 *)(&buf[0]);
    INT32 yCmd = *(INT32 *)(&buf[4]);
    INT32 zCmd = *(INT32 *)(&buf[8]);
    INT32 ori_wCmd = *(INT32 *)(&buf[12]);
    INT32 ori_xCmd = *(INT32 *)(&buf[16]);
    INT32 ori_yCmd = *(INT32 *)(&buf[20]);
    INT32 ori_zCmd = *(INT32 *)(&buf[24]);
    double pos_x = (double)xCmd / 1e6;
    double pos_y = (double)yCmd / 1e6;
    double pos_z = (double)zCmd / 1e6;
    double ori_w = (double)ori_wCmd / 1e6;
    double ori_x = (double)ori_xCmd / 1e6;
    double ori_y = (double)ori_yCmd / 1e6;
    double ori_z = (double)ori_zCmd / 1e6;

    posVector[0] = pos_x;
    posVector[1] = pos_y;
    posVector[2] = pos_z;
    posVector[3] = ori_w;
    posVector[4] = ori_x;
    posVector[5] = ori_y;
    posVector[6] = ori_z;

    pub_localizer_pose(posVector);
    ROS_INFO("GCS recv msg analysis : receive ndt points: x : %f, y : %f, z : %f", pos_x, pos_y, pos_z);
}

static void MessageAnalysis(const char *buf)
{
    UINT8 dir = (UINT8)buf[0];
    UINT8 cmd = (UINT8)buf[1];

    if (dir != FLIGHT_TO_GROUND_STATION)
    { //判断是否为飞机发来的 需要区分
        ROS_ERROR("GCS MessageAnalysis buf[0] isnot 0x02");
        return;
    }

    switch (cmd)
    {
    case MESSAGE_ANALYSIS_RECEIVE_NDTPOINTS:
        ROS_INFO("GCS recv msg analysis : receive ndt points:");
        get_ndt_points(buf + 2);
        break;
    default:
        ROS_INFO("GCS recv msg analysis : unknow cmd - %x", cmd);
        break;
    }
}

static void RecvMsgFromSerialAndAnalyze()
{
    static const int REV_MSG_MAX_LEN = 1024;  //接收消息buffer的最大值
    static char rev_msg_buf[REV_MSG_MAX_LEN]; //得到一帧完整的数据
    static int rev_idx = 0;                   //index
    static int rev_buf_ok = false;            //为false表示消息还没成帧

    int ret;
    size_t nread, offset;
    UINT32 len; //规定了4个字节所以必须为32位

    //得到第一个字符
    ret = Alex_SerialPort_Recv(&rev_msg_buf[rev_idx], 1);
    if (ret == 0)
    { //获取第一个字节超时就不打印了，否则打印的东西太多了
        return;
    }
    else if (ret < 0 || 0xfe != (UINT8)rev_msg_buf[rev_idx])
    {
        ROS_INFO("msg recv : first char - %02x .", (UINT8)rev_msg_buf[rev_idx]);
        ROS_INFO("msg recv : get first char error");
        return;
    }
    rev_idx++;

    //得到第二个字符
    ret = Alex_SerialPort_Recv(&rev_msg_buf[rev_idx], 1);
    if (ret <= 0 || 0xff != (UINT8)rev_msg_buf[rev_idx])
    {
        ROS_INFO("msg recv : second char - %02x .", (UINT8)rev_msg_buf[rev_idx]);
        ROS_INFO("msg recv : get second char error");
        rev_idx = 0;
        return;
    }
    rev_idx++;

    //得到长度
    nread = sizeof(len);
    offset = 0;
    while (nread > 0)
    {
        ret = Alex_SerialPort_Recv((char *)&len + offset, nread);
        if (ret <= 0)
        {
            ROS_ERROR("msg recv : get msg len error");
            rev_idx = 0;
            break;
        }
        nread -= ret;
        offset += ret;
    }
    if (rev_idx == 0)
        return;
    ROS_INFO("msg recv : get len - %u ", len);
    if (len + REV_MSG_HEAD_LEN > REV_MSG_MAX_LEN)
    {
        ROS_ERROR("msg recv : msg len overflow");
        rev_idx = 0;
        return;
    }
    *(UINT32 *)(&rev_msg_buf[rev_idx]) = len;
    rev_idx += sizeof(len);

    //得到数据
    nread = len;
    offset = 0;
    while (nread > 0)
    {
        ret = Alex_SerialPort_Recv(&rev_msg_buf[rev_idx + offset], nread);
        if (ret <= 0)
        {
            ROS_ERROR("msg recv : get msg data error");
            rev_idx = 0;
            break;
        }
        nread -= ret;
        offset += ret;
    }
    if (rev_idx == 0)
        return;
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
    UINT16 crc16_from_msg = *(UINT16 *)(&rev_msg_buf[rev_idx - CRC16_BYTES]);
    UINT16 ctc16_from_calc = crc16(&rev_msg_buf[REV_MSG_HEAD_LEN], len - CRC16_BYTES);

    //打印信息
    //如果是心跳报文则少打印信息
    if (crc16_from_msg == ctc16_from_calc)
    {

        ROS_INFO("rev msg buf: ");

        string str = string(rev_msg_buf, rev_msg_buf + rev_idx);
        PRINT_STRING_TO_BINARY(str);

        ROS_INFO("crc16_from_msg=%p,  ctc16_from_calc=%p", crc16_from_msg, ctc16_from_calc);
    }
    else
    {
        ROS_INFO("rev msg buf: ");

        string str = string(rev_msg_buf, rev_msg_buf + rev_idx);
        PRINT_STRING_TO_BINARY(str);

        ROS_INFO("crc16_from_msg=%p,  ctc16_from_calc=%p", crc16_from_msg, ctc16_from_calc);
        ROS_WARN("crc16 is bad, send msg again!");
        rev_idx = 0;
        return;
    }

    //得到完整的一帧
    rev_buf_ok = true;

    MessageAnalysis(rev_msg_buf + REV_MSG_HEAD_LEN);

    //准备重新获得一帧
    rev_buf_ok = false;
    rev_idx = 0;
}

static void SendBufferQueueToSerialPort()
{
    while (!is_send_buffer_queue_empty())
    {
        pthread_mutex_lock(&send_buffer_queue_lock);
        string tmpstr = send_buffer_queue.front();
        send_buffer_queue.pop();
        pthread_mutex_unlock(&send_buffer_queue_lock);

        /**if ( (UINT8)tmpstr[MSG_TYPE_POS] == (UINT8)MESSAGE_ANALYSIS_HEART_BEAT ) {
            //心跳信息就不打印了，否则日志太多了
            ROS_INFO("send heart beat msg.");
        } else {
            ROS_INFO("send msg_buf: ");
            PRINT_STRING_TO_BINARY(tmpstr);
        }**/

        Alex_SerialPort_Send(tmpstr.c_str(), tmpstr.size());
    }
}

void *GCS_SendAndRecvLoop(void *arg)
{
    while (true)
    {
        RecvMsgFromSerialAndAnalyze();
        SendBufferQueueToSerialPort();
    }
}

void GCS_InitFunc()
{
    pthread_mutex_init(&send_buffer_queue_lock, NULL);
}