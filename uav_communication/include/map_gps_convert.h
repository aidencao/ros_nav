#ifndef MAP_GPS_CON
#define MAP_GPS_TRANSFORM

#include "uavtype.h"

// 百度坐标系转地球坐标系
void bd09_To_Gps84(double in_lat, double in_lon, double &out_lat, double &out_lon);

// 地球坐标系转百度坐标系
void Gps84_To_bd09(double in_lat, double in_lon, double &out_lat, double &out_lon);


#endif

