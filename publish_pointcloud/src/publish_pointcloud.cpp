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

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

using namespace std;

int main(int argc, char **argv)
{

	std::string topic, path, frame_id;
	int hz = 5;
	bool rotate_xyz;

	ros::init(argc, argv, "publish_pointcloud");
	ros::NodeHandle nh;

	nh.param("publish_pointcloud/path", path, std::string("/home/cyr/nav_ws/src/publish_pointcloud/data/hall.pcd"));
	nh.param("publish_pointcloud/frame_id", frame_id, std::string("camera"));
	nh.param("publish_pointcloud/topic", topic, std::string("/pointcloud/output"));
	nh.param("publish_pointcloud/hz", hz, 5);
	nh.param("publish_pointcloud/rotate_xyz", rotate_xyz, false);

	ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(topic, 10);

	pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
	sensor_msgs::PointCloud2 output;
	pcl::io::loadPCDFile(path, cloud1);

	if (rotate_xyz)
	{
		//旋转坐标
		Eigen::Affine3f transform1 = Eigen::Affine3f::Identity();
		transform1.rotate(Eigen::AngleAxisf(1.570795, Eigen::Vector3f::UnitX()));
		transform1.rotate(Eigen::AngleAxisf(1.570795, Eigen::Vector3f::UnitY()));
		pcl::transformPointCloud(cloud1, cloud2, transform1);

		pcl::toROSMsg(cloud2, output); // 转换成ROS下的数据类型 最终通过topic发布
	}
	else
		pcl::toROSMsg(cloud1, output); // 转换成ROS下的数据类型 最终通过topic发布

	output.header.stamp = ros::Time::now();
	output.header.frame_id = frame_id;

	cout << "path = " << path << endl;
	cout << "frame_id = " << frame_id << endl;
	cout << "topic = " << topic << endl;
	cout << "hz = " << hz << endl;

	ros::Rate loop_rate(hz);
	// while (ros::ok())
	// {
	// 	pcl_pub.publish(output);
	// 	ros::spinOnce();
	// 	loop_rate.sleep();
	// }
	loop_rate.sleep();
	pcl_pub.publish(output);
	return 0;
}
