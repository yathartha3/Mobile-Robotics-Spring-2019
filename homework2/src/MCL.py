#!/usr/bin/env python
# Note: some code was adapted from: github yz9/Monte-Carlo-Localization
import rospy
import tf.transformations as tr
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from math import cos, sin
from threading import Lock
import numpy as np
from ParticleFilter import ParticleFilter
import copy


class MonteCarloLocalization:
    def __init__(self, num_particles, xmin, xmax, ymin, ymax):
        
        # Initialize node
        rospy.init_node("MonteCarloLocalization")

        # Get map from map server
        print("Wait for static_map from map server...")
        rospy.wait_for_service('static_map')
        map = rospy.ServiceProxy("static_map", GetMap)
        resp1 = map()
        self.grid_map = resp1.map
        print("Map resolution: " + str(self.grid_map.info.resolution))
        print("Map loaded.")

        self.particle_filter = ParticleFilter(num_particles, self.grid_map,
                                        xmin,xmax,ymin,ymax,
                                        0,0,0,0,
                                        0.25, # translation
                                        0.1,  # orientation
                                        0.3)  # measurement
        self.particle_filter.init_particles()
        self.last_scan = None
        self.mutex = Lock()

        self.particles_pub = rospy.Publisher('/particle_filter/particles', MarkerArray, queue_size=1)
        self.mcl_estimate_pose_pub = rospy.Publisher('/mcl_estimate_pose', PoseStamped, queue_size=1)
        self.publish_fake_scan = rospy.Publisher('/fake_scan', LaserScan, queue_size=1)

        rospy.Subscriber('/base_pose_ground_truth', Odometry, self.odometry_callback, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=1)

        self.first_laser_flag = True
        self.odom_initialized = False
        self.laser_initialized = False

        self.mutex = Lock()

    #********** Callbacks **********#
    def odometry_callback(self, msg):
        self.particle_filter.robot_odom = msg
        if not self.odom_initialized: self.odom_initialized = True

    def laser_callback(self, msg):
        if not self.laser_initialized:
            print("Got first laser callback.")
            self.particle_filter.laser_min_angle = msg.angle_min
            self.particle_filter.laser_max_angle = msg.angle_max
            self.particle_filter.laser_min_range = msg.range_min
            self.particle_filter.laser_max_range = msg.range_max
            self.laser_initialized = True
        self.laser_data = msg

    # -------------------------------- #
    def publish_mcl_estimate_pose(self):
        estimate_pose = PoseStamped()
        # Populate
        estimate_pose.header.stamp = rospy.Time.now()
        estimate_pose.header.frame_id = "map"

        estimate_pose.pose.position.x = self.particle_filter.mean_estimate_particle.x
        estimate_pose.pose.position.y = self.particle_filter.mean_estimate_particle.y

        quat_data = tr.quaternion_from_euler(0,0, self.particle_filter.mean_estimate_particle.theta)
        estimate_pose.pose.orientation.z = quat_data[2]
        estimate_pose.pose.orientation.w = quat_data[3]

        # Publish
        self.mcl_estimate_pose_pub.publish(estimate_pose)

    #********** Publishers ************#
    def publish_particle_markers(self):
        marker_array = MarkerArray()
        timestamp = rospy.Time.now()
        for i in range(len(self.particle_filter.particles)):
            marker_array.markers.append(self.get_rviz_particle_marker(timestamp, self.particle_filter.particles[i], i))
        self.particles_pub.publish(marker_array)
    #----------------------------------#

    def get_rviz_particle_marker(self, timestamp, particle, marker_id):
        """Returns an rviz marker that visualizes a single particle"""
        msg = Marker()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'map'
        msg.ns = 'particles'
        msg.id = marker_id
        msg.type = 0
        msg.action = 0
        msg.lifetime = rospy.Duration(1)

        yaw_in_map = particle.theta
        vx = cos(yaw_in_map)
        vy = sin(yaw_in_map)

        msg.color = ColorRGBA(0, 1.0, 0, 1.0)

        msg.points.append(Point(particle.x, particle.y, 0.2))
        msg.points.append(Point(particle.x + 0.3*vx, particle.y + 0.3*vy, 0.2))

        msg.scale.x = 0.05
        msg.scale.y = 0.05
        msg.scale.z = 0.1
        return msg

    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.publish_particle_markers()
            self.run_mcl()
            rate.sleep()

    def run_mcl(self):
        if self.odom_initialized and self.laser_initialized:
            odom = copy.deepcopy(self.particle_filter.robot_odom)
            laser = copy.deepcopy(self.laser_data)
            # print "old samples"
            self.particle_filter.sample_motion_model_odometry(odom)

            self.particle_filter.handle_observation(laser)

            # Re-sample particles based on weight
            particle_indices = np.arange(self.particle_filter.num_particles)
            new_particles_indices = np.random.choice(particle_indices, size=self.particle_filter.num_particles,
                                            p=self.particle_filter.weights)
            current_particles = copy.deepcopy(self.particle_filter.particles)
            new_particles = [copy.deepcopy(current_particles[i]) for i in new_particles_indices]
            self.particle_filter.particles = copy.deepcopy(new_particles)

            self.particle_filter.calculate_mean_particle()
            self.publish_scan()

            self.publish_mcl_estimate_pose()

    def publish_scan(self):
        ls = LaserScan()
        ls.header.stamp = rospy.Time.now()
        ls.header.frame_id = "base_laser_link"
        ls.angle_min = np.min(self.particle_filter.ls_angles)
        ls.angle_max = np.max(self.particle_filter.ls_angles)
        ls.angle_increment = np.abs(self.particle_filter.ls_angles[0] - self.particle_filter.ls_angles[1])
        ls.range_min = 0.0
        ls.range_max = np.max(self.particle_filter.ls_ranges)
        ls.ranges = self.particle_filter.ls_ranges
        self.publish_fake_scan.publish(ls)


if __name__ == '__main__':
    num_particles = 30
    # Workspace boundaries in meters
    xmin = 0  # for maze map
    xmax = 10
    ymin = 0
    ymax = 10

    mcl = MonteCarloLocalization(num_particles, xmin, xmax, ymin, ymax)
    mcl.run()
