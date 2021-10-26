#! /usr/bin/python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

publisher_old = rospy.Publisher('/laser_scan_old', LaserScan, queue_size = 10)
publisher_new = rospy.Publisher('/laser_scan_new', LaserScan, queue_size = 10)
pub_grid = rospy.Publisher('/occupancy_grid', OccupancyGrid, queue_size = 10)

STEPS = 3
MAX_DISTANCE = 0.08
MAX_FLOAT = 9999

DIMENSIONS = 30
RESOLUTION = 0.1


def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def coordinates(r, theta):
    return r * math.cos(theta), r * math.sin(theta)

def angle(angle_min, angle_increment, position):
    return angle_min + angle_increment * position

def from_polar_to_cart(r, theta):
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    return x,y    

def callback(msg):
    publisher_old.publish(msg)

    msg.ranges = remove_noise(msg.ranges, msg.angle_min, msg.angle_increment)
    publisher_new.publish(msg)

    pub_grid.publish(get_grid_msg(msg.ranges, msg.angle_min, msg.angle_increment))    


def init_grid():
    og_msg = OccupancyGrid()
    width = int(DIMENSIONS / RESOLUTION)
    height = int(DIMENSIONS / RESOLUTION)
    og_msg.header.frame_id = 'base_link'
    og_msg.info.width = width
    og_msg.info.height = height
    og_msg.info.resolution = RESOLUTION
    og_msg.info.origin.position.x = -DIMENSIONS / 2.0
    og_msg.info.origin.position.y = -DIMENSIONS / 2.0
    og_msg.data = [-1] * (width * height)
    return og_msg

def cell(x, y):
    OFFSET_X = 0.25
    OFFSET_Y = -0.025

    cell_x = round((x + OFFSET_X + DIMENSIONS/2) / RESOLUTION) 
    cell_y = round((y + OFFSET_Y + DIMENSIONS/2) / RESOLUTION)

    return cell_x, cell_y

def cast_ray(grid, dist, theta):
    STEP = RESOLUTION / 2.0
    STEPS = dist / STEP
    for i in range(0, int(STEPS)):
        r = dist / STEPS * i
        x, y = from_polar_to_cart(r, theta)
        cell_x, cell_y = cell(x, y)
        index = cell_x + cell_y * int(DIMENSIONS / RESOLUTION)
        if grid[index] == -1:
            grid[index] = 0

def get_grid_msg(ranges, angle_min, angle_step):

    occupancy_grid_msg = init_grid()

    for i, distance in enumerate(ranges):
        if distance > MAX_FLOAT-1.0:
            continue
        angle = angle_min+angle_step*i

        cast_ray(occupancy_grid_msg.data, distance, angle)

        coords = from_polar_to_cart(distance, angle)
        cell_x, cell_y = cell(coords[0], coords[1])
        occupancy_grid_msg.data[cell_x + cell_y * int(DIMENSIONS / RESOLUTION)] = 100

    return occupancy_grid_msg

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

rospy.init_node('lr2')
rospy.Subscriber('base_scan', LaserScan, callback)
rate = rospy.Rate(1)

while not (rospy.is_shutdown()):
    rate.sleep()
