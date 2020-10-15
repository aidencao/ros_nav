    
//============================================================================
// Name        : map_gps_convert.cpp
// Author      : roger
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include "map_gps_convert.h"
#include <math.h>
#include <stdlib.h>


const double pi = 3.14159265358979324;  //圆周率
const double a = 6378245.0;             //卫星椭球坐标投影到平面地图坐标系的投影因子
const double ee = 0.00669342162296594323;   //椭球的偏心率
const double x_pi = 3.14159265358979324 * 3000.0 / 180.0; //圆周率转换量


//判断是否在中国境内
bool outOfChina(double lat, double lon)
{
	if (lon < 72.004 || lon > 137.8347)
		return true;
	if (lat < 0.8293 || lat > 55.8271)
		return true;
	return false;
}

// 加密纬度
double transformLat(double x, double y)
{
	double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * sqrt(abs(x));
	ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
	ret += (20.0 * sin(y * pi) + 40.0 * sin(y / 3.0 * pi)) * 2.0 / 3.0;
	ret += (160.0 * sin(y / 12.0 * pi) + 320 * sin(y * pi / 30.0)) * 2.0 / 3.0;
	return ret;
}

// 加密经度
double transformLon(double x, double y)
{
	double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * sqrt(abs(x));
	ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
	ret += (20.0 * sin(x * pi) + 40.0 * sin(x / 3.0 * pi)) * 2.0 / 3.0;
	ret += (150.0 * sin(x / 12.0 * pi) + 300.0 * sin(x / 30.0 * pi)) * 2.0 / 3.0;
	return ret;
}

/** 
 * 地球坐标转换为火星坐标    
 * World Geodetic System ==> Mars Geodetic System
 * WGS-84 到 GCJ-02 的转换 World Geodetic System ==> Mars Geodetic System
 */
void Gps84_to_Gcj02(double wgLat, double wgLon,double &mgLat,double &mgLon)
{
    if (outOfChina(wgLat, wgLon))
    {
        mgLat  = wgLat;
        mgLon = wgLon;
        return ;
    }
    
    double dLat = transformLat(wgLon - 105.0, wgLat - 35.0);
    double dLon = transformLon(wgLon - 105.0, wgLat - 35.0);
    
    double radLat = wgLat / 180.0 * pi;
    double magic = sin(radLat);
    
    magic = 1 - ee * magic * magic;
    double sqrtMagic = sqrt(magic);
    
    dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * pi);
    dLon = (dLon * 180.0) / (a / sqrtMagic * cos(radLat) * pi);
    
    mgLat = wgLat + dLat;
    mgLon = wgLon + dLon;
	
}

//火星坐标转换为百度坐标 -- GCJ-02 坐标转换成 BD-09 坐标
void Gcj02_To_bd09(double in_lat, double in_lon,double &out_lat,double &out_lon)
{
    double x = in_lon, y = in_lat;
    double z = sqrt(x * x + y * y) + 0.00002 * sin(y * x_pi);
    double theta = atan2(y, x) + 0.000003 * cos(x * x_pi);
    out_lon = z * cos(theta) + 0.0065;
    out_lat = z * sin(theta) + 0.006;
}

//百度坐标系转火星坐标系
void bd09_To_Gcj02(double in_lat, double in_lon, double &out_lat, double &out_lon)
{
    double x = in_lon - 0.0065;
    double y = in_lat - 0.006;
    double z = sqrt(x * x + y * y) - 0.00002 * sin(y * x_pi);
    double theta = atan2(y, x) - 0.000003 * cos(x * x_pi);
    out_lon = z * cos(theta);
    out_lat = z * sin(theta);
}

// 火星坐标系转地球坐标系
void Gcj02_To_Gps84(double in_lat, double in_lon, double &out_lat, double &out_lon)
{
    double g_x, g_y;
    Gps84_to_Gcj02(in_lat, in_lon, g_x, g_y);

    out_lat = in_lat * 2 - g_x;
    out_lon = in_lon * 2 - g_y;
}


// 百度坐标系转地球坐标系
void bd09_To_Gps84(double in_lat, double in_lon, double &out_lat, double &out_lon)
{
    double marLat, marLon;
    bd09_To_Gcj02(in_lat, in_lon, marLat, marLon);
    Gcj02_To_Gps84(marLat, marLon, out_lat, out_lon);
}

// 地球坐标系转百度坐标系
void Gps84_To_bd09(double in_lat, double in_lon, double &out_lat, double &out_lon)
{
    double marLat, marLon;
    Gps84_to_Gcj02(in_lat, in_lon, marLat, marLon);
    Gcj02_To_bd09(marLat, marLon, out_lat, out_lon);
}



