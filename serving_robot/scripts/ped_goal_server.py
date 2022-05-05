#!/usr/bin/env python

from __future__ import print_function

from serving_robot.srv import GetPedPose,GetPedPoseResponse
from zed_interfaces.msg import ObjectsStamped
from geometry_msgs.msg import Pose
import rospy

class PedPoseServer():
    def __init__(self):
        rospy.init_node('get_ped_pose_server')
        self.ped_data_sub = rospy.Subscriber("/zed2i/zed_node/obj_det/objects", ObjectsStamped, self.ped_data_callback)
        self.ped_data_publisher = rospy.Publisher("/ped_pose/goal", Pose, queue_size=1)
        self.ped_data_object = Pose()
        self.ped_data_dict = {}

    def ped_data_callback(self, data):
        for i in range(len(data.objects)):
            self.ped_data_dict[data.objects[i].label_id] = [data.objects[i].position[0], data.objects[i].position[1]]
            self.ped_pose_publisher()

    def ped_pose_publisher(self):
        goal_location = self.goal_selector()
        self.ped_data_object.position.x = goal_location[0]
        self.ped_data_object.position.y = goal_location[1]
        self.ped_data_publisher.publish(self.ped_data_object)


    def goal_selector(self):
        # This function will have the logic for locking the pedestrian and setting goal location
        keys = self.ped_data_dict.keys()
        return self.ped_data_dict[keys[0]]


    def handle_get_ped_pose(self, req):
        goal_location = self.goal_selector()
        ped_pose = Pose()
        ped_pose.position.x = goal_location[0]
        ped_pose.position.y = goal_location[1]
        return GetPedPoseResponse(ped_pose)


    def get_ped_pose_server(self):
        s = rospy.Service('get_ped_pose', GetPedPose, self.handle_get_ped_pose)
        print("Ready to provide ped pose.")
        rospy.spin()

if __name__ == "__main__":
    p = PedPoseServer()
    p.get_ped_pose_server()