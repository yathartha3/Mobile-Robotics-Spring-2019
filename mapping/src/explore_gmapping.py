#!/usr/bin/env python

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import random
import rospy
import copy
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid

class Explore():
    def __init__(self):
        rospy.init_node("explore_node")
        self.current_map = None
        self.map_flag = True  # first callback flag

        # subscribe to map
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=1)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server(rospy.Duration(10))

    def map_callback(self, map_msg):
        self.current_map = map_msg
        if self.map_flag:
            self.map_resolution = map_msg.info.resolution
            self.map_width = map_msg.info.width
            self.map_height = map_msg.info.height
            self.map_origin = map_msg.info.origin
            self.map_flag = False

    def run_explore(self):
        if self.current_map!=None:
            _map = copy.deepcopy(self.current_map)
            grid_map = _map
            grid_map = np.reshape(_map.data, (self.map_width, self.map_height), order='F')

            unexplored_list = []
            for w in range(self.map_width):
                for h in range(self.map_height):
                    if grid_map[w][h] == -1:
                        unexplored_list.append([w,h])

            next_pos_pixel = np.array(random.choice(unexplored_list))
            next_pos = next_pos_pixel*self.map_resolution # multiply array elements with a scalar

            next_pos[0] = next_pos[0] + self.map_origin.position.x
            next_pos[1] = next_pos[1] + self.map_origin.position.y

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
    rospy.sleep(5) # wait for things to load up
    explorer = Explore()

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        explorer.run_explore()
        rate.sleep()



