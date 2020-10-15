#ifndef UAV_HEART_BEAT
#define UAV_HEART_BEAT

#include "uavtype.h"
#include "uav_inc.h"



class HeartBeat
{
public:
    static void init();
    static void *thread_loop(void *arg);
    static bool is_heart_beat_start_flag_ok();
    static bool turn_on_heart_beat_start_flag();
    static bool turn_off_heart_beat_start_flag();
    static bool is_heart_beat_come_flag_on();
    static void turn_on_heart_beat_come_flag();    
    static void turn_off_heart_beat_come_flag();
private:    
    HeartBeat();
    ~HeartBeat();    
    //心跳处理是否开始
    static bool heart_beat_start_flag;         
    static pthread_mutex_t heart_beat_start_flag_lock;
    //这1秒内是否接收到过心跳报文或者其他报文
    static bool heart_beat_come_flag;          
    static pthread_mutex_t heart_beat_come_flag_lock;
};

#endif
