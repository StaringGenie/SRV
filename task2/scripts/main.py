#! /usr/bin/python3

import rospy
import math
from sensor_msgs.msg import LaserScan

publisher_old = rospy.Publisher('/laser_scan_old', LaserScan, queue_size = 10)
publisher_new = rospy.Publisher('/laser_scan_new', LaserScan, queue_size = 10)

STEPS = 3
MAX_DISTANCE = 0.05
MAX_FLOAT = 9999


def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def coordinates(r, theta):
    return r * math.cos(theta), r * math.sin(theta)

def angle(angle_min, angle_increment, position):
    return angle_min + angle_increment * position

def callback(msg):
    publisher_old.publish(msg)
    msg.ranges = remove_noise(msg.ranges, msg.angle_min, msg.angle_increment)
    publisher_new.publish(msg)


def remove_noise(ranges, angle_min, angle_increment):
    result = []

    for i in range(STEPS):
        result.append(MAX_FLOAT)

    for i in range(STEPS, len(ranges) - STEPS):
        prev_point = coordinates(ranges[i - STEPS], angle(angle_min, angle_increment, i - STEPS))
        current_point = coordinates(ranges[i], angle(angle_min, angle_increment, i))
        next_point = coordinates(ranges[i + STEPS], angle(angle_min, angle_increment, i + STEPS))

        if distance(prev_point, current_point) > MAX_DISTANCE or distance(next_point, current_point) > MAX_DISTANCE:
            result.append(MAX_FLOAT)
        else:
            result.append(ranges[i])

    for i in range(STEPS):
        result.append(MAX_FLOAT)

    return result

rospy.init_node('lr1_normal')
rospy.Subscriber('base_scan', LaserScan, callback)
rate = rospy.Rate(1)

while not (rospy.is_shutdown()):
    rate.sleep()
