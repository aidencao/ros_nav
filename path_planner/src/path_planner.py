#!/usr/bin/env python
# coding:utf-8
import numpy as np
import fcl
import rospy
from octomap_msgs.msg import Octomap
import math


def isCollision():
    box1 = fcl.Box(1, 1, 1)
    t1 = fcl.Transform()
    o1 = fcl.CollisionObject(box1, t1)

    box2 = fcl.Box(1, 1, 1)
    t2 = fcl.Transform()
    o2 = fcl.CollisionObject(box2, t2)

    request = fcl.CollisionRequest()
    result = fcl.CollisionResult()

    fcl.collide(o1, o2, request, result)

    return result.is_collision

# def getOctomapCB(data):

if __name__ == '__main__':
    try:
        x = 0
        y = 0
        z = 0
        w = 1
        yaw = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))
        angleYaw = yaw*180/math.pi
        print(yaw)
    except KeyboardInterrupt:
        print("Shutting down path_planner")
