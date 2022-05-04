#!/usr/bin/env python

from __future__ import print_function

from serving_robot.srv import GetPedPose,GetPedPoseResponse
from zed_interfaces.msg import ObjectsStamped
import rospy

def handle_get_ped_pose(req):
    print("returning ped pose")
    return None

def get_ped_pose_server():
    rospy.init_node('get_ped_pose_server')
    s = rospy.Service('get_ped_pose', GetPedPose, handle_get_ped_pose)
    print("Ready to provide ped pose.")
    rospy.spin()

if __name__ == "__main__":
    get_ped_pose_server()