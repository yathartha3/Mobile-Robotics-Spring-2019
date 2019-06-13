#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
import csv


import tf.transformations as tr
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from math import cos, sin
import numpy as np
import copy
import cv2 as cv


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

        self.circles_pub = rospy.Publisher('/detected_circles', MarkerArray, queue_size=1)

        rospy.Timer(rospy.Duration(1), self.timer_callback)

        self.laser_data = None
        self.robot_odom = None

        self.laser_initialized = None
        self.grid_size = 0.01  # should ideally come from map

        self.circle_locations = []

        self.failed_goal_index = []
        self.debug = False

        rospy.Subscriber('/base_pose_ground_truth', Odometry, self.odometry_callback, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=1)

        print(self.failed_goal_index)
        print("Initialized")

        #self.movebase_client()

    def timer_callback(self, event):
        print("timer called")
        self.detect_shape()

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

    def heading(self, yaw):
        q = quaternion_from_euler(0, 0, yaw)
        return Quaternion(*q)

    def distance(self, p1, p2):
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def read_waypoints_from_csv(self, filename):
        # Import waypoints.csv into a list (path_points)
        poses_waypoints = []

        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f, delimiter=' ')]
        path_points = [(float(point[0]), float(point[1]), float(point[2])) for point in path_points]
        #print path_points
        for point in path_points:
            waypoint = Pose(Point(float(point[0]), float(point[1]), 0), self.heading(float(point[2])))
            poses_waypoints.append(waypoint)
        return poses_waypoints

    def laser_to_image(self, odom_msg, laser_msg):
        """ Convert 1-D laser data to 2-D image data so that Hough Transform Filter can be applied to it"""
        robot_x = odom_msg.pose.pose.position.x
        robot_y = odom_msg.pose.pose.position.y

        quaternion = np.array([odom_msg.pose.pose.orientation.x,
                              odom_msg.pose.pose.orientation.y,
                              odom_msg.pose.pose.orientation.z,
                              odom_msg.pose.pose.orientation.w])
        _, _, robot_theta = tr.euler_from_quaternion(quaternion)

        occupied_pixel_list_xy = []

        for i in range(len(laser_msg.ranges)):
            scan_range = laser_msg.ranges[i]  # range measured for the particular scan
            scan_angle = laser_msg.angle_min + i*laser_msg.angle_increment  # bearing measured

            # scans at the max range are not obstacles
            if scan_range == laser_msg.range_max:
                continue

            # find position of cells in the global frame
            occupied_x = scan_range * cos(robot_theta + scan_angle) + robot_x
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

        # Create blank image of the map
        image = np.zeros([1000, 1000])

        # Populate the occupied pixels
        for p in occupied_pixel_list_xy:
            image[p[0], p[1]] = 255  # is this correct

        # Run it through circle detection filter and get circle centers
        center_x, center_y = self.hough_filter(image)

        if len(center_x) != 0:
            # Convert pixels to map
            center_x = center_x * self.grid_size
            center_y = center_y * self.grid_size

            # NOTE: center_x contains all x's, and center_y contains all y's. Need to arrage to get point (x,y)
            # for n in range(len(center_x)):
            #     self.circle_locations.append([center_x[n], center_y[n]])

            self.publish_all_circles(copy.deepcopy(self.circle_locations))

    def publish_all_circles(self, circles_location):
        marker_array = MarkerArray()
        timestamp = rospy.Time.now()

        for i in range(len(circles_location)):
            marker_array.markers.append(self.get_rviz_cylinder_marker(timestamp, circles_location[i], i))

        self.circles_pub.publish(marker_array)

    def get_rviz_cylinder_marker(self, timestamp, circle, marker_id):
        """Returns an rviz marker that visualizes a single particle"""
        msg = Marker()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'map'
        msg.ns = 'circles'
        msg.id = marker_id
        msg.type = 3
        msg.action = 0
        #msg.lifetime = rospy.Duration(1)

        msg.color = ColorRGBA(0, 1.0, 0, 1.0)
        msg.pose.position = Point(circle[1], circle[0], 0.2)  # NOTE: Reversed

        msg.scale.x = 0.25  # 50 cm diameter circles
        msg.scale.y = 0.25
        msg.scale.z = 0.1
        return msg

    def get_rviz_cylinder_marker(self, timestamp, circle, marker_id):
        """Returns an rviz marker that visualizes a single particle"""
        msg = Marker()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'map'
        msg.ns = 'circles'
        msg.id = marker_id
        msg.type = 3
        msg.action = 0
        #msg.lifetime = rospy.Duration(1)

        msg.color = ColorRGBA(0, 1.0, 0, 1.0)
        msg.pose.position = Point(circle[1], circle[0], 0.2)  # NOTE: Reversed

        msg.scale.x = 0.25  # 50 cm diameter circles
        msg.scale.y = 0.25
        msg.scale.z = 0.1
        return msg

    def check_circle_duplicate(self, test_circle):
        if len(self.circle_locations) == 0:
            return False
        for existing_circle in self.circle_locations:
            if self.distance(test_circle, existing_circle) < 0.5:
                return True
        return False

    def pixel_to_metric(self, x, y):
        x_m = x * self.grid_size + 0  # Zero map offset
        y_m = y * self.grid_size + 0
        return x_m, y_m

    def hough_filter(self, image):
        image = image.astype(np.uint8)
        kernel = np.ones((3, 3), np.float32) / 4
        image = cv.filter2D(image, -1, kernel)
        circles = cv.HoughCircles(image, cv.HOUGH_GRADIENT, 1, 10,
                                  param1=10, param2=8,
                                  minRadius=23, maxRadius=27)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for circle in circles[0, :]:
                center = (circle[0], circle[1])

                test_circle = self.pixel_to_metric(circle[0],circle[1])
                if not self.check_circle_duplicate(test_circle):
                    self.circle_locations.append(test_circle)

                # circle center
                cv.circle(image, center, 1, (0, 100, 100), 3)
                # circle outline
                radius = circle[2]
                #print "radius=", radius
                cv.circle(image, center, radius, (255, 0, 255), 3)

            #cv.imshow("detected circles", image)

            return circles[0][:,0], circles[0][:,1]  # all x's, and all y's
        return [], []  # If nothing is found

    def detect_shape(self):
        laser_data = copy.deepcopy(self.laser_data)
        odom_data = copy.deepcopy(self.robot_odom)

        laser_image = self.laser_to_image(odom_data, laser_data)

        #print("Detecting shape")
        # Convert to 2D, and pass it through filter
        # First convert all laser points to world coordinates, and then to pixels

    # Action Client Callbacks
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

            #self.detect_shape()

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
    rate = rospy.Rate(1)

    circle_detector = MoveBaseWaypoints()
    rospy.sleep(2)

    while not rospy.is_shutdown():
        #circle_detector.detect_shape()
        print("running")
        circle_detector.movebase_client()
        rate.sleep()
