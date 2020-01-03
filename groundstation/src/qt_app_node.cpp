#include "mainwindow.h"
#include <QApplication>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <signal.h>
#include <sstream>

ros::Publisher basic_cmd_pub;
ros::Publisher drone_height_pub;
ros::Publisher gps_point_pub;
void signalhandler(int sig)
{
    qApp->quit();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "qt_groundstation", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    basic_cmd_pub = nh.advertise<std_msgs::Int8>("control_cmd", 10);
    drone_height_pub = nh.advertise<std_msgs::Float32>("drone_height_cmd", 10);
    //rviz shows gps point
    gps_point_pub = nh.advertise<std_msgs::String>("gps_point_cmd", 10);
    QApplication a(argc, argv);

    signal(SIGQUIT, signalhandler);
    signal(SIGINT, signalhandler);
    signal(SIGTERM, signalhandler);
    signal(SIGHUP, signalhandler);
    signal(SIGKILL, signalhandler);

    MainWindow w;
    w.show();



    /**ros::Rate loop_rate(1); //1hz
    while(ros::ok)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }*/

    return  a.exec();
}


void send_take_off_cmd()
{
    std_msgs::Int8 take_off;
    ROS_INFO("SEND TAKE_OFF CMD");
    take_off.data = 11; //take_off
    basic_cmd_pub.publish(take_off);
}

void send_land_cmd()
{
    std_msgs::Int8 land;
    ROS_INFO("SEND LAND CMD");
    land.data = 12;
    basic_cmd_pub.publish(land);
}

void send_drone_height_cmd(const float drone_height)
{
    ROS_INFO("SEND DRONE HEIGHT, VALUE IS : %f", drone_height);
    std_msgs::Float32 drone_height_cmd;
    drone_height_cmd.data = drone_height;
    drone_height_pub.publish(drone_height_cmd);
}

void send_gps_height_and_number(const std::string cmd)
{
    ROS_INFO("gps number and height is  : %s", cmd.c_str());
    std_msgs::String msg;
    msg.data = cmd;
    gps_point_pub.publish(msg);
}
