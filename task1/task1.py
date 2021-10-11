#! /usr/bin/python3

import rospy
import math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class Task1:
	def __init__(self):
		rospy.Subscriber('/turtle1/pose', Pose, self.callback1)
		rospy.Subscriber('/turtle2/pose', Pose, self.callback2)
		self.x = 0
		self.y = 0
		self.theta = 0
		self.publisher = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size = 1)

	def callback1(self, msg):
		message = Twist()
		if self.is_nearby(msg):
			return
		message.linear.x = math.sqrt((msg.x - self.x) ** 2 + (msg.y - self.y) ** 2) / 5
		message.angular.z = math.atan2(msg.y - self.y, msg.x - self.x) - self.theta
		self.publisher.publish(message)

	def callback2(self, msg):
		self.x = msg.x
		self.y = msg.y
		self.theta = msg.theta

	def is_nearby(self, msg):
		return abs(self.x - msg.x) < 0.1 and abs(self.y - msg.y) < 0.1

rospy.init_node('turtle_0')
Task1()
rospy.spin()