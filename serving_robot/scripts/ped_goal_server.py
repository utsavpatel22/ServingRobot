#!/usr/bin/env python

from __future__ import print_function

from serving_robot.srv import GetPedPose,GetPedPoseResponse
from zed_interfaces.msg import ObjectsStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rospy
import tf
import math

class PedPoseServer():
    def __init__(self):
        rospy.init_node('get_ped_pose_server')
        self.ped_data_sub = rospy.Subscriber("/zed2i/zed_node/obj_det/objects", ObjectsStamped, self.ped_data_callback)
        # self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.amcl_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)
        self.ped_data_publisher = rospy.Publisher("/goal_ped_pose", PoseStamped, queue_size=1)
        self.ped_data_object = PoseStamped()
        self.ped_data_dict = {}
        self.listener = tf.TransformListener()
        # self.odom_data = Odometry()
        self.amcl_pose_data = PoseWithCovarianceStamped()

    def ped_data_callback(self, data):
        self.ped_data_dict = {}
        for i in range(len(data.objects)):
            self.ped_data_dict[data.objects[i].label_id] = [data.objects[i].position[0], data.objects[i].position[1]]
            self.ped_pose_publisher()

    # Lets not use it       
    # def odom_callback(self, data):
    #     self.odom_data.pose.pose.orientation.x = data.pose.pose.orientation.x 
    #     self.odom_data.pose.pose.orientation.y = data.pose.pose.orientation.y
    #     self.odom_data.pose.pose.orientation.z = data.pose.pose.orientation.z
    #     self.odom_data.pose.pose.orientation.w = data.pose.pose.orientation.w
        
    def amcl_pose_callback(self,data):
        self.amcl_pose_data.pose.pose.orientation.x = data.pose.pose.orientation.x
        self.amcl_pose_data.pose.pose.orientation.y = data.pose.pose.orientation.y
        self.amcl_pose_data.pose.pose.orientation.z = data.pose.pose.orientation.z
        self.amcl_pose_data.pose.pose.orientation.w = data.pose.pose.orientation.w


    def ped_pose_publisher(self):
        goal_location = self.goal_selector()
        if abs(goal_location[0]) < 0.0001:
            goal_location[0] = 0.0001
        ped_yaw = math.atan(goal_location[1]/goal_location[0]) # get ped_yaw in robot frame
        rob_q = self.amcl_pose_data.pose.pose.orientation 
        (_, _, robot_yaw) = tf.transformations.euler_from_quaternion([rob_q.x, rob_q.y, rob_q.z, rob_q.w])
        
        self.ped_data_object.pose.position.x = goal_location[0] - (1 * math.cos(ped_yaw)) #subtracting 1 meter to keep the robot at a safe distance
        self.ped_data_object.pose.position.y = goal_location[1] - (1 * math.sin(ped_yaw))
        self.ped_data_object.header.frame_id = "base_link"
        
        goal_in_map = self.listener.transformPose("map", self.ped_data_object)
        (x,y,z,w) = tf.transformations.quaternion_from_euler(0, 0, ped_yaw + robot_yaw) # gives goal yaw in map frame
        goal_in_map.pose.orientation.x = x
        goal_in_map.pose.orientation.y = y
        goal_in_map.pose.orientation.z = z
        goal_in_map.pose.orientation.w = w
        
        print("Ped_in_rob: ", round(math.degrees(ped_yaw)), "\t Rob_in_map: ", round(math.degrees(robot_yaw)))

        self.ped_data_publisher.publish(goal_in_map)


    def goal_selector(self):
        # This function will have the logic for locking the pedestrian and setting goal location
        keys = list(self.ped_data_dict.keys())
        return self.ped_data_dict[keys[0]]


    def handle_get_ped_pose(self, req):
        goal_location = self.goal_selector()
        ped_pose = PoseStamped()
        ped_pose.pose.position.x = goal_location[0]
        ped_pose.pose.position.y = goal_location[1]
        ped_pose.header.frame_id = "base_link"
        return GetPedPoseResponse(ped_pose)


    def get_ped_pose_server(self):
        s = rospy.Service('get_ped_pose', GetPedPose, self.handle_get_ped_pose)
        print("Ready to provide ped pose.")
        rospy.spin()

if __name__ == "__main__":
    p = PedPoseServer()
    p.get_ped_pose_server()
