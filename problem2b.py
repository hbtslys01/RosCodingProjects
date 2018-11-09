#!/usr/bin/env python2
# coding=utf8
import rospy
import random
import numpy as np
import time
import math
import std_msgs.msg as std_msgs
import std_srvs.srv as std_srvs
from geometry_msgs.msg import Twist,Pose,Quaternion,Point
import p4.msg as p4_msg
import p4.srv as p4_srv
from tf.transformations import *

class WaypointFollower:

    def __init__(self):
        rospy.init_node('problem2b')
	rospy.wait_for_service('/closest_point_path')
	self.closestpoint = rospy.ServiceProxy('/closest_point_path', p4_srv.ClosestPointPath )
	self.vel_publisher = rospy.Publisher('/vrep/youbot/base/cmd_vel', Twist, queue_size=10)
	self.rate = rospy.Rate(10)

   
        rospy.Service('problem2b', p4_srv.FollowPath, self.callback)
	"""self.pos_subscriber = rospy.Subscriber('/vrep/youbot/base/pose', Pose, self.getpos)
	self.pos = Point()
	self.ori = Quaternion()
    def getpos(self,data):
	self.pos = data.position
	self.ori = data.orientation"""
  
    def callback(self, data):
	print("enter callback")
	print("path is : ",data.path)
	vx = 4.
	pose = rospy.wait_for_message('/vrep/youbot/base/pose',Pose)
	pos = pose.position #current position
	while abs(data.path[-1].x - pos.x) > 0.1 : # and abs(data.path[-1].y - pos.y) > 0.1:
	 	pose = rospy.wait_for_message('/vrep/youbot/base/pose',Pose)
		pos = pose.position #current position
		ori = pose.orientation #current orientation in quaternion
		orilist=[ori.x,ori.y,ori.z,ori.w]
		#print(ori)
		goal = self.closestpoint(pos,data.path)
		"""if goal.closest_point.x < pos.x:
			goal.closest_point.x += pos.x - goal.closest_point.x + data.path[-1]- 
			print("back")"""
		print("goal is: ", goal.closest_point)
		print("current posi is: ", pos)
		original = np.matrix([[1],[0],[0],[1]])
		rot = quaternion_matrix(orilist)
		ori_vector = (rot * original)[0:2] # current orientation(x) in vector
		goal_vector = np.matrix([goal.closest_point.x - pos.x,goal.closest_point.y - pos.y]) # vector oh in global frame
		x_h_b = goal_vector * ori_vector
		k = float(2.*np.sqrt(np.square(goal.dist_to_path.data) - np.square(x_h_b))/np.square(goal.dist_to_path.data))
		print("kerv is: ",k)
		angular = k * vx 
		pub = Twist()
		pub.linear.x = vx
		pub.angular.z = angular
		print("velo is : ", pub)
		self.vel_publisher.publish(pub)
		"""old_y = pos.y
		
		pose = rospy.wait_for_message('/vrep/youbot/base/pose',Pose)
		pos = pose.position #current position
		
		hile abs(pos.y - goal.closest_point.y) < abs(old_y - goal.closest_point.y):
			old_y = pos.y
			self.vel_publisher.publish(pub)
			pose = rospy.wait_for_message('/vrep/youbot/base/pose',Pose)
			pos = pose.position #current position"""
	
	return []
if __name__ == '__main__':
    waypoint_follower = WaypointFollower()
    rospy.spin()
