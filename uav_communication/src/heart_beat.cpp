/*
 *         这是心跳控制的代码
 */

#include "heart_beat.h"
#include <ros/ros.h>
#include "serial_port_send_thread.h"


bool HeartBeat::heart_beat_start_flag = false;         
pthread_mutex_t HeartBeat::heart_beat_start_flag_lock;

bool HeartBeat::heart_beat_come_flag = false;          
pthread_mutex_t HeartBeat::heart_beat_come_flag_lock;


bool HeartBeat::is_heart_beat_start_flag_ok()
{
    pthread_mutex_lock(&heart_beat_start_flag_lock);
    bool ret = heart_beat_start_flag;
    pthread_mutex_unlock(&heart_beat_start_flag_lock);
    return ret;
}

bool HeartBeat::turn_on_heart_beat_start_flag()
{
    pthread_mutex_lock(&heart_beat_start_flag_lock);
    bool flag = heart_beat_start_flag;
    heart_beat_start_flag = true;
    pthread_mutex_unlock(&heart_beat_start_flag_lock);
    return flag;
}

bool HeartBeat::turn_off_heart_beat_start_flag()
{
    pthread_mutex_lock(&heart_beat_start_flag_lock);
    bool flag = heart_beat_start_flag;
    heart_beat_start_flag = false;
    pthread_mutex_unlock(&heart_beat_start_flag_lock);  
    return flag;
}

bool HeartBeat::is_heart_beat_come_flag_on()
{
    pthread_mutex_lock(&heart_beat_come_flag_lock);
    bool ret = heart_beat_come_flag;
    pthread_mutex_unlock(&heart_beat_come_flag_lock);
    return ret;
}


void HeartBeat::turn_on_heart_beat_come_flag()
{
    pthread_mutex_lock(&heart_beat_come_flag_lock);
    heart_beat_come_flag = true;
    pthread_mutex_unlock(&heart_beat_come_flag_lock);
}

void HeartBeat::turn_off_heart_beat_come_flag()
{
    pthread_mutex_lock(&heart_beat_come_flag_lock);
    heart_beat_come_flag = false;
    pthread_mutex_unlock(&heart_beat_come_flag_lock);
}


void* HeartBeat::thread_loop(void *arg)
{
    const int HEART_BEAT_MISS_COUNT_MAX = 10;
    int heart_beat_miss_count = 0; //丢失心跳次数
    
    while (1)
    {
        sleep(1);

        //判断是否要开始处理心跳
        if ( ! is_heart_beat_start_flag_ok() ) continue;

        //要发送信息过去，否则心跳可能还是会断
        SerialPortSend::SendHeartBeatMsg();
        
        //1秒内收到过心跳报文或者其他报文
        //在HEART_BEAT_MISS_COUNT_MAX次数内都可以挽回
        if ( is_heart_beat_come_flag_on() )
        {
            heart_beat_miss_count = 0;
            turn_off_heart_beat_come_flag();
            continue;
        }
        //本次没收到心跳报文或者其他报文
        else {
            heart_beat_miss_count++;

            //丢失心跳次数到达上限
            if (heart_beat_miss_count == HEART_BEAT_MISS_COUNT_MAX)
            {
                //此时先暂停维持心跳
                ROS_INFO("turn_off_heart_beat_start_flag");
                heart_beat_miss_count = 0;
                turn_off_heart_beat_start_flag();            
                //turn_off_heart_beat_come_flag();
            }
        }
        
    }
}

void HeartBeat::init()
{
    //
    heart_beat_start_flag = false;
    heart_beat_come_flag = false; 
    //初始化锁
    pthread_mutex_init(&heart_beat_start_flag_lock, NULL);
    pthread_mutex_init(&heart_beat_come_flag_lock, NULL);
}



