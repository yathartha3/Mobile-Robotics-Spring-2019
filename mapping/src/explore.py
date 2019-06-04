#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
import csv
from nav_msgs.msg import Odometry
import random
import copy
import rospy
import tf.transformations as tr
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import LaserScan
from math import cos, sin
import numpy as np
import copy
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion

class Explore():

    def __init__(self):
        rospy.init_node("explore_node")
        self.current_map = None
        self.map_flag = True  # first callback flag

        # subscribe to map
        rospy.Subscriber('/ogm_map', OccupancyGrid, self.map_callback, queue_size=1)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server(rospy.Duration(10.0))

    def map_callback(self, map_msg):
        self.current_map = map_msg
        if self.map_flag:
            self.map_resolution = map_msg.info.resolution
            self.map_width = map_msg.info.width
            self.map_height = map_msg.info.height
            self.map_flag = False

    def run_explore(self):
        if self.current_map!=None:
            _map = copy.deepcopy(self.current_map)
            grid_map = np.reshape(_map.data, (self.map_width, self.map_height), order='F')

            unexplored_list = []

            for w in range(self.map_width):
                for h in range(self.map_height):
                    if grid_map[w][h] == 50:
                        unexplored_list.append([w,h])

            next_pos_pixel = np.array(random.choice(unexplored_list))
            next_pos = next_pos_pixel*self.map_resolution # multiply array elements with a scalar

            self.send_waypoint(next_pos)

    def send_waypoint(self, next_pos_pixel):
        next_goal = MoveBaseGoal()
        next_goal.target_pose.header.frame_id = "map"
        next_goal.target_pose.header.stamp = rospy.Time.now()
        next_goal.target_pose.pose = Pose(Point(next_pos_pixel[0], next_pos_pixel[1], 0), Quaternion(0,0,0,1))

        rospy.loginfo("Sending goal pose " + str(next_pos_pixel) + " to Action Server")
        self.client.send_goal(next_goal)
        result = self.client.wait_for_result(rospy.Duration(30))

        if not result:
            self.client.cancel_goal()


if __name__=="__main__":
    rospy.sleep(1) # wait for things to load up
    explorer = Explore()

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        explorer.run_explore()
        rate.sleep()



