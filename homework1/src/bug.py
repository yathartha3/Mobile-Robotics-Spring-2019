#!/usr/bin/env python

# Every python controller needs these lines
import rospy
# The velocity command message
from geometry_msgs.msg import Twist, PoseStamped, Point, Vector3, Pose, Quaternion
# The laser scan message
from sensor_msgs.msg import LaserScan

from std_msgs.msg import String, Header, ColorRGBA
import math
import tf.transformations as transform
from nav_msgs.msg import Odometry
import numpy as np
import copy
from visualization_msgs.msg import Marker

class BugPlanner():
    def __init__(self):
        rospy.init_node('bug_node')
        self.L_min_dist = 10
        self.F_min_dist = 10
        
        # Subscribers and Publishers
        rospy.Subscriber('scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/base_pose_ground_truth', Odometry, self.gt_callback)
        rospy.Subscriber("/clicked_goal", PoseStamped, self.goal_callback)
        self.vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.goal_marker_pub = rospy.Publisher('goal_marker', Marker, queue_size=10)
        self.path_marker_pub = rospy.Publisher('path_marker', Marker, queue_size=10)
        
        self.OBS_DETECT_DIST = 0.75  # front
        self.GOAL = np.array([10,0,0])
        self.MAX_LIN_VEL = 0.2
        self.GOAL_TOLERANCE = 0.1
        self.RVIZ_DISP_FLAG = True
        self.WALL_KEEPOUT = self.OBS_DETECT_DIST
        #self.ORG_HEADING = self.calc_goal_heading(self.ORIGIN, self.GOAL)

        self.curr_position = np.array([0,0,0])
        self.curr_orientation = 0  # must be in RADIANS

        self.wall_angle = 0

        # Controller Parameters
        self.KP_WALL_DIST = 0
        self.KP_ANGLE = 0.25

        self.obs_in_front = False
        self.obs_position = np.array([0,0,0])

        self.re_goal = True


        rospy.Timer(rospy.Duration(0.2), self.rviz_display_callback)  


    def to_cartesian(self, radius, theta):
        return np.array([radius * math.cos(theta), radius * math.sin(theta), 0.0])

    def calc_l_wall_angle(self, msg):
        # sample_1_range = msg.ranges[532]
        # sample_2_range = msg.ranges[639]
        # sample_1_pos = np.array([self.curr_position[0]+math.cos(math.radians(60)+self.curr_orientation)*sample_1_range, self.curr_position[0]+math.sin(math.radians(60)+self.curr_orientation)*sample_1_range])
        # sample_2_pos = np.array([self.curr_position[1]+math.cos(math.radians(90)+self.curr_orientation)*sample_2_range, self.curr_position[1]+math.sin(math.radians(90)+self.curr_orientation)*sample_2_range])
        # wall_direction = sample_1_pos-sample_2_pos
        # self.wall_angle = math.atan2(wall_direction[1], wall_direction[0])

        p1 = self.curr_position + self.to_cartesian(msg.ranges[639], math.radians(90) + self.curr_orientation)
        p2 = self.curr_position + self.to_cartesian(msg.ranges[533], math.radians(60) + self.curr_orientation)
        wall_dir = p2 - p1
        self.wall_angle = math.atan2(wall_dir[1], wall_dir[0])

    def calc_goal_heading(self, curr, target):  # actually it is vector pointiing from one point to another
        return math.atan2(target[1]-curr[1], target[0]-curr[0])

    def laser_callback(self, msg):
        """
        Calculate min distance from left, and forward.
        Calculate angle of left wall based on two points.
        """
        # self.goal_dir_lines = []
        # self.goal_dir_lines.append(Point(self.curr_position[0], self.curr_position[1], self.curr_position[2]))
        # self.goal_dir_lines.append(Point(self.GOAL[0], self.GOAL[1], self.GOAL[2]))

        self.L_min_dist = min(msg.ranges[533:640])
        self.F_min_dist = min(msg.ranges[285:355])
        self.calc_l_wall_angle(msg)

    def goal_callback(self, msg):
        self.GOAL = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
        self.goal_dir_lines = []
        self.goal_dir_lines.append(Point(self.curr_position[0], self.curr_position[1], self.curr_position[2]))
        self.goal_dir_lines.append(Point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))

    def rviz_display_callback(self, event):
        if self.RVIZ_DISP_FLAG:
            self.view_rviz()

    def view_rviz(self):
        goal_marker = Marker()
        goal_marker.type=Marker.ARROW
        goal_marker.id=0
        goal_marker.lifetime=rospy.Duration(0.2)
        goal_marker.pose.position=Point(self.GOAL[0],self.GOAL[1],self.GOAL[2])
        goal_marker.scale=Vector3(0.5, 0.05, 0.05)
        goal_marker.header=Header(frame_id='map')
        goal_marker.color=ColorRGBA(1.0, 0.0, 0.0, 0.5)
        line_marker = Marker(
            type=Marker.LINE_STRIP,
            id=2,
            lifetime=rospy.Duration(0.2),
            points=self.goal_dir_lines,
            scale=Vector3(0.06, 0.06, 0.06),
            header=Header(frame_id='map'),
            color=ColorRGBA(0.0, 0.0, 1.0, 0.5))
        self.goal_marker_pub.publish(goal_marker)
        self.path_marker_pub.publish(line_marker)


    def gt_callback(self, msg):
        odom_pos = msg.pose.pose.position
        quaternion_data = msg.pose.pose.orientation
        self.curr_position = np.array([odom_pos.x, odom_pos.y, 0])
        quat = (quaternion_data.x, quaternion_data.y, quaternion_data.z, quaternion_data.w)
        self.curr_orientation = transform.euler_from_quaternion(quat)[2]

        if self.re_goal:
            self.goal_dir_lines = []
            self.goal_dir_lines.append(Point(self.curr_position[0], self.curr_position[1], self.curr_position[2]))
            self.goal_dir_lines.append(Point(self.GOAL[0], self.GOAL[1], self.GOAL[2]))
            self.re_goal = False


    def align_w_wall(self):
        self.publish(0, -0.2)#self.KP_ANGLE*(math.radians(90)-self.wall_angle))

    def follow_wall(self):
        control_rate = rospy.Rate(20)
        print("Wall angle:" + str(math.degrees(self.wall_angle)))
        while (self.F_min_dist<=self.OBS_DETECT_DIST) or (abs(self.curr_orientation - self.wall_angle) > 0.15):
            # print()
            # print(math.degrees(self.curr_orientation))
            # print(math.degrees(self.wall_angle))
            # print(math.degrees(self.curr_orientation-self.wall_angle))
            self.align_w_wall()
        #input("asdf")
        print("Following")

        while not rospy.is_shutdown():
            dist_err = (self.L_min_dist - self.WALL_KEEPOUT)
            angl_err = (self.wall_angle - self.curr_orientation)

            control_dist = self.KP_WALL_DIST*dist_err
            control_angl = self.KP_ANGLE*angl_err
            print(control_angl)

            # self.publish(self.MAX_LIN_VEL, control_dist)
            # control_rate.sleep()
            self.publish(self.MAX_LIN_VEL, control_angl)
            control_rate.sleep()

            curr_heading = self.calc_goal_heading(self.curr_position, self.GOAL)
            original_heading = self.calc_goal_heading(self.obs_position, self.GOAL)

            if abs(curr_heading-original_heading) < math.radians(3):  # heading actually means vector from one point to another, not robot's heading
                if self.distance(self.obs_position, self.curr_position)>0.5:
                    while (np.abs(self.curr_orientation - original_heading))>0.1:
                        self.align_w_wall() #actually not align with wall
                    self.obs_in_front = False
                    print("Follow Vector Again")
                    return

    def follow_goal_vector(self):
        # Get current heading
        goal_heading = self.calc_goal_heading(self.curr_position, self.GOAL)
        goal_heading_err = goal_heading - self.curr_orientation

        # print("\nGoal heading      :" + str(goal_heading))
        # print("Goal heading Error:" + str(goal_heading_err))

        # Check if wall in front
        if self.F_min_dist < self.OBS_DETECT_DIST:
            print(self.F_min_dist)
            self.obs_in_front = True
            self.obs_position = copy.deepcopy(self.curr_position)
            return

        if goal_heading_err>math.radians(5):  # Maneuver for high heading error
            self.publish(0, goal_heading_err)
        else:
            self.publish(self.MAX_LIN_VEL, goal_heading_err*self.KP_ANGLE)
        self.obs_in_front = False


    def publish(self, linear_vel, angular_vel):
        command = Twist()
        command.linear.x = linear_vel
        command.angular.z = angular_vel
        # Publish the command using the global publisher
        self.vel_pub.publish(command)

    def distance(self, P1, P2):
        return np.linalg.norm(P1-P2)


if __name__ == '__main__':
    rospy.sleep(10.0)  # wait for things to load up
    bug_planner = BugPlanner()

    while not rospy.is_shutdown():
        if bug_planner.distance(bug_planner.curr_position, bug_planner.GOAL) < bug_planner.GOAL_TOLERANCE:
            print("GOAL REACHED")
            break

        bug_planner.follow_goal_vector()
        if bug_planner.obs_in_front:
            bug_planner.follow_wall()

    rospy.spin()
