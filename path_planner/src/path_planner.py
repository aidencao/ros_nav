#!/usr/bin/env python
# coding:utf-8
import numpy as np
import fcl
import rospy
from octomap_msgs.msg import Octomap


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
        rospy.init_node("path_planner_node")

        print(isCollision())
    except KeyboardInterrupt:
        print("Shutting down path_planner")
