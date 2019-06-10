#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
import csv
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import random
import copy
import numpy as np
from math import sin, cos
import tf.transformations as tr

import matplotlib.pyplot as plt

from skimage import data, color
from skimage.transform import hough_circle, hough_circle_peaks
from skimage.feature import canny
from skimage.draw import circle_perimeter
from skimage.util import img_as_ubyte
from skimage.io import imread, imshow


"""Some code adapted from https://hotblackrobotics.github.io/en/blog/2018/01/29/seq-goals-py/"""
"""This code is also part of our project."""


class MoveBaseWaypoints():
    def __init__(self):
        # Create action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server(rospy.Duration(5))
        self.goal_cnt = 0
        filename = rospy.get_param('~waypoints_filepath', '')
        self.waypoints = self.read_waypoints_from_csv(filename)

        rospy.Subscriber('/base_pose_ground_truth', Odometry, self.odometry_callback, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=1)

        self.laser_data = None
        self.robot_odom = None

        self.laser_initialized = None
        self.grid_size = 0.01  # should ideally come from map

        self.circle_locations = []

        self.start_flag = True
        self.start_x = None
        self.start_y = None
        self.failed_goal_index = []
        print(self.failed_goal_index)
        print("Initialized")

        #self.movebase_client()

    def odometry_callback(self, msg):  # call only once
        self.robot_odom = msg

    def laser_callback(self, msg):
        if not self.laser_initialized:
            print("Got first laser callback.")
            self.laser_min_angle = msg.angle_min
            self.laser_max_angle = msg.angle_max
            self.laser_min_range = msg.range_min
            self.laser_max_range = msg.range_max
            self.laser_initialized = True
        self.laser_data = msg

##### DEBUGGING
        self.detect_shape()

    def heading(self, yaw):
        q = quaternion_from_euler(0, 0, yaw)
        return Quaternion(*q)

    def read_waypoints_from_csv(self, filename):
        # Import waypoints.csv into a list (path_points)
        poses_waypoints = []

        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f, delimiter=' ')]
        path_points = [(float(point[0]), float(point[1]), float(point[2])) for point in path_points]
        print path_points
        for point in path_points:
            waypoint = Pose(Point(float(point[0]), float(point[1]), 0), self.heading(float(point[2])))
            poses_waypoints.append(waypoint)
        return poses_waypoints

    ### Shape detection and processing functions

    def laser_to_image(self, odom_msg, laser_msg):
        """ Convert 1-D laser data to 2-D image data so that Hough Transform Filter can be applied to it"""
        robot_x = odom_msg.pose.pose.position.x
        robot_y = odom_msg.pose.pose.position.y

        self.robot_x = robot_x
        self.robot_y = robot_y

        quaternion = np.array([odom_msg.pose.pose.orientation.x,
                              odom_msg.pose.pose.orientation.y,
                              odom_msg.pose.pose.orientation.z,
                              odom_msg.pose.pose.orientation.w])
        _, _, robot_theta = tr.euler_from_quaternion(quaternion)

        #robot_x_pixel = int(robot_x/self.grid_size)
        #robot_y_pixel = int(robot_y/self.grid_size)

        # # Bound it between [200,200], the map size in pixels
        # robot_x_pixel = max(0, min(999, robot_x_pixel))
        # robot_y_pixel = max(0, min(999, robot_y_pixel))

        max_x = 0
        max_y = 0

        occupied_pixel_list_xy = []

        for i in range(len(laser_msg.ranges)):
            scan_range = laser_msg.ranges[i]  # range measured for the particular scan
            scan_angle = laser_msg.angle_min + i*laser_msg.angle_increment  # bearing measured

            # scans at the max range are not obstacles
            if scan_range == laser_msg.range_max:
                continue

            # find position of cells in the global frame
            occupied_x = scan_range * cos(robot_theta + scan_angle) + robot_x # sth funky, should've been cos i thought
            occupied_y = scan_range * sin(robot_theta + scan_angle) + robot_y

            negative = False
            if (occupied_x < 0) or (occupied_y < 0):
                negative = True
            if negative == True:
                continue

            pixel_occupied_x = int(occupied_x/self.grid_size)
            pixel_occupied_y = int(occupied_y/self.grid_size)

            pixel_occupied_x = max(0, min(999, pixel_occupied_x))
            pixel_occupied_y = max(0, min(999, pixel_occupied_y))

            occupied_pixel_list_xy = occupied_pixel_list_xy + [(pixel_occupied_x, pixel_occupied_y)]

            if pixel_occupied_x > max_x: max_x = pixel_occupied_x
            if pixel_occupied_y > max_y: max_y = pixel_occupied_y

        # Create blank image of reduced size
        image = np.zeros([max_x+1, max_y+1])

        # Populate the occupied pixels
        for p in occupied_pixel_list_xy:
            image[p[0], p[1]] = 255  # is this correct

        # Run it through circle detection filter
        center_x, center_y = self.hough_filter(image)
        self.circle_locations.append([center_x, center_y])

        print()

    def hough_filter(self, image):
        edges = canny(image, sigma=1)

        # Detect two radii
        hough_radii = np.arange(30, 100, 2)  # These params work
        hough_res = hough_circle(edges, hough_radii)

        # Select the most prominent 5 circles, cx,cy are the centers
        accums, cx, cy, radii = hough_circle_peaks(hough_res, hough_radii,
                                                   total_num_peaks=2,
                                                   min_xdistance=25,
                                                   min_ydistance=25)
        # Draw them
        fig, ax = plt.subplots(ncols=1, nrows=1, figsize=(10, 4))
        image = color.gray2rgb(image) * 0

        for center_y, center_x, radius in zip(cy, cx, radii):
            circy, circx = circle_perimeter(center_y, center_x, radius)
            image[circy, circx] = (220, 20, 20)

        ax.imshow(image, cmap=plt.cm.gray)
        plt.show()

        return cx, cy

    def detect_shape(self):
        laser_data = copy.deepcopy(self.laser_data)
        odom_data = copy.deepcopy(self.robot_odom)

        laser_image = self.laser_to_image(odom_data, laser_data)

        # Convert to 2D, and pass it through filter
        # First convert all laser points to world coordinates, and then to pixels

    ### Action Client Callbacks

    def active_cb(self):
        rospy.loginfo("Goal pose " + str(self.goal_cnt + 1) + " is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        rospy.loginfo("Feedback for goal pose " + str(self.goal_cnt + 1) + " received")
        # print(feedback)

    def skip_waypoint(self):
        print("Skipping, or Looping waypoint")
        next_goal = MoveBaseGoal()
        next_goal.target_pose.header.frame_id = "map"
        next_goal.target_pose.header.stamp = rospy.Time.now()
        next_goal.target_pose.pose = self.waypoints[self.goal_cnt]
        rospy.loginfo("Sending goal pose " + str(self.goal_cnt + 1) + " to Action Server")
        rospy.loginfo(str(self.waypoints[self.goal_cnt]))
        self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)

    def done_cb(self, status, result):
        '''
        Keeps track of which goals failed.
        Generates the next goal index randomly from waypoints range, with failed goal indices removed.
        '''
        if status == 2:
            rospy.loginfo("Goal pose " + str(
                self.goal_cnt) + " received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose " + str(self.goal_cnt + 1) + " reached")
            self.goal_cnt += 1

            self.detect_shape()

            if self.goal_cnt < len(self.waypoints):
                rospy.sleep(2.0)
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.waypoints[self.goal_cnt]
                rospy.loginfo("Sending goal pose " + str(self.goal_cnt + 1) + " to Action Server")
                rospy.loginfo(str(self.waypoints[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)
            else:
                print("\nRe-looping")
                print("Waypoints reversed\n")
                self.waypoints.reverse()

                self.goal_cnt = 1  # waypoints have been reversed

                self.skip_waypoint()
                return

        if status == 4:
            rospy.loginfo("Goal pose " + str(self.goal_cnt) + " was aborted by the Action Server")
            self.failed_goal_index.append(self.goal_cnt)
            print("Goal " + str(self.goal_cnt) + " appended to Failed Goals Index List")

            self.waypoints.remove(self.waypoints[self.goal_cnt])
            print("Bad waypoint removed from waypoints list!")

            # self.goal_cnt += 1
            # dont need to increment here because the index was removed
            self.skip_waypoint()
            return

        if status == 5:
            rospy.loginfo("Goal pose " + str(self.goal_cnt) + " has been rejected by the Action Server")
            self.failed_goal_index.append(self.goal_cnt)

            self.goal_cnt += 1
            # self.check_goal()
            self.skip_waypoint()
            return

        if status == 8:
            rospy.loginfo("Goal pose " + str(
                self.goal_cnt) + " received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.waypoints[self.goal_cnt]
        rospy.loginfo("Sending goal pose " + str(self.goal_cnt + 1) + " to Action Server")
        rospy.loginfo(str(self.waypoints[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)

        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('explore_find_circles_node')
    rate = rospy.Rate(10)
    try:
        MoveBaseWaypoints()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
    while not rospy.is_shutdown():
        rate.sleep()
