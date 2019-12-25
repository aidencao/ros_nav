#include <ros/ros.h>
#include <serial/serial.h> //ROS已经内置了的串口包
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
serial::Serial ser; //声明串口对象
//回调函数
void write_callback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data); //发送串口数据
}

float bytes2float(uint8_t *bytes)
{
    float temp = 0;
    int n = sizeof(temp);
    memccpy(&temp, bytes, 'i' ,n);
    return temp;
}

double getdata(int start, int end, uint8_t *buffer)
{
    uint8_t buffer_t[4];
    for(int i = start; i<end; i++)
    {
        buffer_t[i-start] = buffer[i];
    }
    double temp = (double)bytes2float(buffer_t);
    return temp;
}

bool checkLrc(uint8_t *buffer)
{
    uint8_t low = 0;
    uint8_t high = 0;
    
    for(int i = 1; i<=86; i++)
    {
        if(i%2 == 0)
        {
            high = high + buffer[i];
        }
        else
        {
            low = low + buffer[i];
        }
    }

    if((low == buffer[87]) && (high == buffer[88]))
    {
        return true;
    }
    else
    {
        std::cout<< std::hex << (low & 0xff) << " "<< std::hex << (high & 0xff) << " " << std::hex << (buffer[87] & 0xff) << " "<< std::hex << (buffer[88] & 0xff) << " "<< std::endl;
        return false;
    }
}

sensor_msgs::Imu setImuData(uint8_t *buffer)
{
    //写入时间戳
    sensor_msgs::Imu imu_data;
    imu_data.header.stamp = ros::Time::now();
    imu_data.header.frame_id = "base_link";

    //写入四元数
    imu_data.orientation.w = getdata(47,51,buffer);
    imu_data.orientation.x = getdata(51,55,buffer);
    imu_data.orientation.y = getdata(55,59,buffer);
    imu_data.orientation.z = getdata(59,63,buffer);
    ROS_INFO_STREAM("Serial Port 1");

    //写入线性加速度
    imu_data.linear_acceleration.x = getdata(75,79,buffer);
    imu_data.linear_acceleration.y = getdata(79,83,buffer);
    imu_data.linear_acceleration.z = getdata(83,87,buffer);
    ROS_INFO_STREAM("Serial Port 2");

    //写入角速度
    imu_data.angular_velocity.x = getdata(11,15,buffer);
    imu_data.angular_velocity.y = getdata(15,19,buffer);
    imu_data.angular_velocity.z = getdata(19,23,buffer);
    ROS_INFO_STREAM("Serial Port 3");

    return imu_data;
}

int main(int argc, char **argv)
{
    //初始化节点
    ros::init(argc, argv, "serial_example_node");
    //声明节点句柄
    ros::NodeHandle nh;
    //订阅主题，并配置回调函数
    //ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    //发布主题
    ros::Publisher read_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 1);
    try
    {
        //设置串口属性，并打开串口
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    //检测串口是否已经打开，并给出提示信息
    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }
    //指定循环的频率
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        //获取缓冲区的字节数
        size_t n = ser.available();

        if (n!=0)
        {
            uint8_t buffer[1024];
            //读取数据
            ser.flush();
            n = ser.read(buffer , 91);
            if(((int)buffer[0] == 0x3a) && ((int)buffer[1] == 0x1))
            {
                read_pub.publish(setImuData(buffer));
                ROS_INFO_STREAM("Serial Port 4");
            }
            else
            {
                ROS_INFO_STREAM("Serial read error");
            }
            
            //   for (int i=0; i<n; i++)
            //   {
            //         //16
            //         std::cout << std::hex << (buffer[i] & 0xff) << " ";
            //   }
            //   std::cout << std::endl;

        }

        //处理ROS的信息，比如订阅消息,并调用回调函数
        //ros::spinOnce();
        loop_rate.sleep();
        ROS_INFO_STREAM("Serial Port 5");
    }
}
