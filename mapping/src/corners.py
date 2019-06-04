#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
import csv

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
        self.movebase_client()

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

    def done_cb(self, status, result):
        self.goal_cnt += 1
        if status == 2:
            rospy.loginfo("Goal pose " + str(
                self.goal_cnt) + " received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose " + str(self.goal_cnt) + " reached")
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
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                rospy.sleep(5.0)
                return

        if status == 4:
            rospy.loginfo("Goal pose " + str(self.goal_cnt) + " was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose " + str(self.goal_cnt) + " aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose " + str(self.goal_cnt) + " has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose " + str(self.goal_cnt) + " rejected, shutting down!")
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
