/**
*
* 函数功能:读取pcl点云文件并发布到topic上去
*
*
* maker: crp 
* data: 2016-6-8
*/

#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>
#include <stdlib.h>

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

using namespace std;

float getAngle(float rad)
{
    return M_PI / 180 * rad;
}

void SplitString(const string &s, vector<string> &v, const string &c)
{
    string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while (string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2 - pos1));

        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if (pos1 != s.length())
        v.push_back(s.substr(pos1));
}

void readTxt(string file, pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    ifstream infile;
    infile.open(file.data()); //将文件流对象与文件连接起来
    assert(infile.is_open()); //若失败,则输出错误消息,并终止程序运行

    string s;
    while (getline(infile, s))
    {
        vector<string> v;
        double x, y, z;
        SplitString(s, v, ",");
        x = stof(v[0]);
        y = stof(v[1]);
        z = stof(v[2]);
        pcl::PointXYZ point = {x, y, z};
        cloud.points.push_back(point);
    }
    infile.close(); //关闭文件输入流
}

int main(int argc, char **argv)
{

    std::string topic, load_path, output_path, frame_id, point_path;

    double rotatex, rotatey, rotatez, movex, movey, movez;

    ros::init(argc, argv, "rotate_map");
    ros::NodeHandle nh;

    nh.param("rotate_map/load_path", load_path, std::string("/home/cyr/nav_ws/src/rotate_map/data/hall.pcd"));
    nh.param("rotate_map/output_path", output_path, std::string("/home/cyr/nav_ws/src/rotate_map/data/hall.pcd"));
    nh.param("rotate_map/frame_id", frame_id, std::string("camera"));
    nh.param("rotate_map/topic", topic, std::string("/pointcloud/output"));
    nh.param("rotate_map/rotatex", rotatex, 0.0);
    nh.param("rotate_map/rotatey", rotatey, 0.0);
    nh.param("rotate_map/rotatez", rotatez, 0.0);
    nh.param("rotate_map/movex", movex, 0.0);
    nh.param("rotate_map/movey", movey, 0.0);
    nh.param("rotate_map/movez", movez, 0.0);
    nh.param("rotate_map/point_path", point_path, std::string("/home/cyr/nav_ws/src/rotate_map/data/point.txt"));

    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(topic, 1);

    //标定点数据
    pcl::PointCloud<pcl::PointXYZ> pointcloud1, pointcloud2;
    readTxt(point_path, pointcloud1);

    //地图数据
    pcl::PointCloud<pcl::PointXYZ>
        cloud1, cloud2;
    sensor_msgs::PointCloud2 output;
    pcl::io::loadPCDFile(load_path, cloud1);

    //旋转坐标
    Eigen::Affine3f transform1 = Eigen::Affine3f::Identity();
    transform1.rotate(Eigen::AngleAxisf(getAngle(rotatex), Eigen::Vector3f::UnitX()));
    transform1.rotate(Eigen::AngleAxisf(getAngle(rotatey), Eigen::Vector3f::UnitY()));
    transform1.rotate(Eigen::AngleAxisf(getAngle(rotatez), Eigen::Vector3f::UnitZ()));

    transform1.translation() << movex, movey, movez;

    pcl::transformPointCloud(cloud1, cloud2, transform1);
    pcl::transformPointCloud(pointcloud1, pointcloud2, transform1);

    //写入文件
    ofstream out(point_path);
    for (size_t i = 0; i < pointcloud2.points.size(); i++)
    {
        out << pointcloud2.points[i].x << "," << pointcloud2.points[i].y << "," << pointcloud2.points[i].z << endl;
    }
    out.close();

    pcl::toROSMsg(cloud2, output); // 转换成ROS下的数据类型 最终通过topic发布

    pcl::io::savePCDFile(output_path, cloud2);

    output.header.stamp = ros::Time::now();
    output.header.frame_id = frame_id;

    pcl_pub.publish(output);
    return 0;
}