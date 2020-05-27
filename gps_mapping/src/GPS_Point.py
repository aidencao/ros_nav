#!/usr/bin/env python
# coding:utf-8
class GPS_Point:
    # 构造函数
    def __init__(self, x, y, lat, lon):
        self.x = x
        self.y = y
        self.lat = lat
        self.lon = lon

    def getx(self):
        return self.x

    def gety(self):
        return self.y

    def getLat(self):
        return self.lat

    def getLon(self):
        return self.lon

    def printInfo(self):
        print(self.x + ' ' + self.y + ' ' + self.lat + ' ' + self.lon)


# 读取所有的点位
def readPoint(path):
    points = []
    with open(path) as lines:
        for line in lines:
            point = line.split()
            xyz = point[0].split(',')
            gps = point[1].split(',')

            gpspoint = GPS_Point(float(xyz[0]), float(xyz[1]), float(gps[0]),
                                 float(gps[1]))
            points.append(gpspoint)

    return points


# def x_latitudeMapping(x1, lat1, x2, lat2):  # 纬度向北增加 经度往东增加 假定x轴与纬度重合
#     a = (lat1 - lat2) / (x1 - x2)
#     b = lat1 - a * x1
#     return a, b

# def y_longitudeMapping(y1, lon1, y2, lon2):  # 经度向东增加 假定y轴负方向与经度重合
#     a = (lon1 - lon2) / (y1 - y2)
#     b = lon1 - a * y1
#     return a, b


def get_lon(x0, x1, x2, y0, y1, y2, lon0, lon1, lon2):
    a1 = x1 - x0
    a2 = x2 - x0
    b1 = y1 - y0
    b2 = y2 - y0
    c1 = lon1 - lon0
    c2 = lon2 - lon0

    d = a1 * b2 - b1 * a2
    e = c1 * b2 - b1 * c2
    f = a1 * c2 - c1 * a2

    A = e / d
    B = f / d

    return A, B


def get_lat(x0, x1, x2, y0, y1, y2, lat0, lat1, lat2):
    a1 = x1 - x0
    a2 = x2 - x0
    b1 = y1 - y0
    b2 = y2 - y0
    c1 = lat1 - lat0
    c2 = lat2 - lat0

    d = a1 * b2 - b1 * a2
    e = c1 * b2 - b1 * c2
    f = a1 * c2 - c1 * a2

    C = e / d
    D = f / d

    return C, D


# 建立平均值映射关系
def avg_mapping(points):
    latalist = []
    latblist = []
    lonalist = []
    lonblist = []

    point0 = points[0]

    for p in [points[i:i + 2] for i in range(1, len(points), 2)]:
        lata, latb = get_lat(point0.getx(), p[0].getx(), p[1].getx(),
                             point0.gety(), p[0].gety(), p[1].gety(),
                             point0.getLat(), p[0].getLat(), p[1].getLat())
        lona, lonb = get_lon(point0.getx(), p[0].getx(), p[1].getx(),
                             point0.gety(), p[0].gety(), p[1].gety(),
                             point0.getLon(), p[0].getLon(), p[1].getLon())
        latalist.append(lata)
        latblist.append(latb)
        lonalist.append(lona)
        lonblist.append(lonb)

    # for p in [points[i:i + 2] for i in range(0, len(points), 2)]:
    #     xa, xb = x_latitudeMapping(p[0].getx(), p[0].getLat(), p[1].getx(),
    #                                p[1].getLat())
    #     ya, yb = y_longitudeMapping(p[0].gety(), p[0].getLon(), p[1].gety(),
    #                                 p[1].getLon())
    #     xalist.append(xa)
    #     xblist.append(xb)
    #     yalist.append(ya)
    #     yblist.append(yb)

    avglata = sum(latalist) / len(latalist)
    avglatb = sum(latblist) / len(latblist)
    avglona = sum(lonalist) / len(lonalist)
    avglonb = sum(lonblist) / len(lonblist)

    return avglata, avglatb, avglona, avglonb, point0


# def trans(x, y):
#     global avglata, avglatb, avglona, avglonb, point0

    # lat = point0.getLat() + avglata * (x - point0.getx()) + avglatb * (
    #     y - point0.gety())
    # lon = point0.getLon() + avglona * (x - point0.getx()) + avglonb * (
    #     y - point0.gety())

#     return lat, lon

# if __name__ == '__main__':
#     points = readPoint('/home/cyr/nav_ws/src/gps_mapping/src/test.txt')

#     avg_mapping(points)

#     lat, lon = trans(41.4466667175, -0.747732877731)
#     print("lat:" + str(lat) + "  lon:" + str(lon))
