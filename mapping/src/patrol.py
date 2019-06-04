#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
import csv
from nav_msgs.msg import Odometry
import random

"""Some code adapted from https://hotblackrobotics.github.io/en/blog/2018/01/29/seq-goals-py/"""
"""This code is also part of our project."""

class MoveBaseWaypoints():
    def __init__(self):
        # Create action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server(rospy.Duration(10.0))
        self.goal_cnt = 0
        filename = rospy.get_param('~waypoints_filepath', '')
        self.waypoints = self.read_waypoints_from_csv(filename)
        
        #rospy.Subscriber('/base_pose_ground_truth', Odometry, self.odometry_callback, queue_size=1)
        self.start_flag = True
        self.start_x = None
        self.start_y = None
        self.failed_goal_index = []
        print(self.failed_goal_index)
        print("Initialized")
        self.movebase_client()

    def odometry_callback(self, msg):  # call only once
        if start_flag:
            self.start_x = msg.pose.pose.position.x
            self.start_y = msg.pose.pose.position.y
            start_flag = False
            print("Saved starting position")

    def heading(self,yaw):
        q = quaternion_from_euler(0, 0, yaw)
        return Quaternion(*q)

    def read_waypoints_from_csv(self,filename):
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

    def active_cb(self):
        rospy.loginfo("Goal pose " + str(self.goal_cnt + 1) + " is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        rospy.loginfo("Feedback for goal pose " + str(self.goal_cnt + 1) + " received")
        #print(feedback)

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
            rospy.loginfo("Goal pose " + str(self.goal_cnt+1) + " reached")
            self.goal_cnt += 1


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

                self.goal_cnt = 1 # waypoints have been reversed

                self.skip_waypoint()
                return

        if status == 4:
            rospy.loginfo("Goal pose " + str(self.goal_cnt) + " was aborted by the Action Server")
            self.failed_goal_index.append(self.goal_cnt)
            print("Goal "+str(self.goal_cnt)+" appended to Failed Goals Index List")

            self.waypoints.remove(self.waypoints[self.goal_cnt])
            print("Bad waypoint removed from waypoints list!")

            #self.goal_cnt += 1
            # dont need to increment here because the index was removed
            self.skip_waypoint()
            return

        if status == 5:
            rospy.loginfo("Goal pose " + str(self.goal_cnt) + " has been rejected by the Action Server")
            self.failed_goal_index.append(self.goal_cnt)

            self.goal_cnt += 1
            #self.check_goal()
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
    rospy.init_node('corner_node')
    rate = rospy.Rate(10)
    try:
        MoveBaseWaypoints()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
    while not rospy.is_shutdown():
        rate.sleep()
