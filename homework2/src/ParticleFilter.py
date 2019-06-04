#!/usr/bin/env python
import tf.transformations as tr
from math import cos, sin
from math import pi, exp
import random
import numpy as np


class Particle:
    def __init__(self, i, x, y, theta):
        self.x = x; self.y = y; self.theta = theta
        self.id = i


class ParticleFilter:
    def __init__(self, num_particles, occ_grid_map, xmin, xmax, ymin, ymax,
                 laser_min_range, laser_max_range, laser_min_angle, laser_max_angle,
                 dynamics_translation_noise_std_dev,
                 dynamics_orientation_noise_std_dev,
                 beam_range_measurement_noise_std_dev):
        self.num_particles = num_particles
        self.ogm = occ_grid_map
        self.grid_map = np.array(self.ogm.data, dtype='int8')
        self.grid_map = self.grid_map.reshape((self.ogm.info.height, self.ogm.info.width))
        self.grid_bin = (self.grid_map == 0).astype('uint8')  # Cell is True iff probability of being occupied is zero
        # grid bin is of size [200,200]
        
        # Workspace boundaries
        self.xmax = xmax
        self.xmin = xmin
        self.ymin = ymin
        self.ymax = ymax

        self.laser_max_angle = laser_max_angle
        self.laser_min_angle = laser_min_angle
        self.laser_max_range = laser_max_range
        self.laser_min_range = laser_min_range

        # Std deviation of noise affecting translation, orientation and range (for laser measurement model)
        self.dynamics_translation_noise_std_dev = dynamics_translation_noise_std_dev
        self.dynamics_orientation_noise_std_dev = dynamics_orientation_noise_std_dev
        self.beam_range_measurement_noise_std_dev = beam_range_measurement_noise_std_dev

        # Number of laser beams for sensor model
        self.eval_beams = 32

        # Previous odometry measurement of the robot
        self.last_robot_odom = None

        # Current odometry measurement of the robot
        self.robot_odom = None

        # Relative motion since the last time particles were updated
        self.dx = 0
        self.dy = 0
        self.dyaw = 0

        self.particles = []  # predicted set or corrected set?
        self.weights = [(1.0/self.num_particles) for _ in range(self.num_particles)]

        # Particle estimate
        self.mean_estimate_particle = Particle(9999, 2.0, 2.0, 0.0) # id, x, y, theta

        self.ls_ranges = []
        self.ls_angles = []

    def metric_to_grid_coords(self, x, y):
        """Converts metric coordinates to occupancy grid coordinates"""
        gx = (x - self.ogm.info.origin.position.x) / self.ogm.info.resolution  # map resolution [m/cell]
        gy = (y - self.ogm.info.origin.position.y) / self.ogm.info.resolution
        
        row = min(max(int(gy), 0), self.ogm.info.height)
        col = min(max(int(gx), 0), self.ogm.info.width)
        return (row, col)

    def get_random_free_state(self):
        while True:
            # Samling around start estimate
            epsilon = 1
            xrand = np.random.uniform(2-epsilon, 2+epsilon)
            yrand = np.random.uniform(2-epsilon, 2+epsilon)

            row, col = self.metric_to_grid_coords(xrand, yrand)
            if self.grid_bin[row, col]:
                theta = 0 +np.random.uniform(0, pi/4.0)#pi/4.0)
                return xrand, yrand, theta

    def init_particles(self):
        """Initializes particles uniformly randomly with map frame coordinates,
        within the boundaries set by xmin,xmax, ymin,ymax"""
        for i in range(self.num_particles):
            xrand, yrand, theta = self.get_random_free_state()
            # Note: same orientation as the initial orientation of the robot to make initialization easier
            self.particles.append(Particle(i, xrand, yrand, theta))
        print("Particles Initialized.")

    # ----- Motion Update Functions ----- #
    def sample_motion_model_odometry(self, robot_odom):
        """Control Ut=[old_odom, new_odom]. We also have all the particle poses"""
        # Update robot odom

        # last_theta = 0
        # current_theta = 0
        #
        #
        # d_rot1 = 0
        # d_trans = 0
        # d_rot2 = 0

        # TODO: refactor this into it's own function
        if self.last_robot_odom:
            last_quat = np.array([self.last_robot_odom.pose.pose.orientation.x,
                                           self.last_robot_odom.pose.pose.orientation.y,
                                           self.last_robot_odom.pose.pose.orientation.z,
                                           self.last_robot_odom.pose.pose.orientation.w])
            current_quat = np.array([self.robot_odom.pose.pose.orientation.x,
                                           robot_odom.pose.pose.orientation.y,
                                           robot_odom.pose.pose.orientation.z,
                                           robot_odom.pose.pose.orientation.w])
            _, _, last_theta = tr.euler_from_quaternion(last_quat)
            _, _, current_theta = tr.euler_from_quaternion(current_quat)

            p_map_currbaselink = np.array([robot_odom.pose.pose.position.x,
                                           robot_odom.pose.pose.position.y,
                                           robot_odom.pose.pose.position.z])

            p_map_lastbaselink = np.array([self.last_robot_odom.pose.pose.position.x,
                                           self.last_robot_odom.pose.pose.position.y,
                                           self.last_robot_odom.pose.pose.position.z])

            q_map_lastbaselink = np.array([self.last_robot_odom.pose.pose.orientation.x,
                                           self.last_robot_odom.pose.pose.orientation.y,
                                           self.last_robot_odom.pose.pose.orientation.z,
                                           self.last_robot_odom.pose.pose.orientation.w])

            q_map_currbaselink = np.array([robot_odom.pose.pose.orientation.x,
                                           robot_odom.pose.pose.orientation.y,
                                           robot_odom.pose.pose.orientation.z,
                                           robot_odom.pose.pose.orientation.w])

            R_map_lastbaselink = tr.quaternion_matrix(q_map_lastbaselink)[0:3, 0:3]

            p_lastbaselink_currbaselink = R_map_lastbaselink.transpose().dot(
                p_map_currbaselink - p_map_lastbaselink)
            q_lastbaselink_currbaselink = tr.quaternion_multiply(tr.quaternion_inverse(q_map_lastbaselink),
                                                                 q_map_currbaselink)

            _, _, yaw_diff = tr.euler_from_quaternion(q_lastbaselink_currbaselink)

            self.dyaw = yaw_diff
            self.dx = p_lastbaselink_currbaselink[0]  # these are in the particle's local frame
            self.dy = p_lastbaselink_currbaselink[1]

            #self.dx = self.robot_odom.pose.pose.position.x - self.last_robot_odom.pose.pose.position.x
            #self.dy = self.robot_odom.pose.pose.position.y - self.last_robot_odom.pose.pose.position.y

            # Lines 2-4 of the algorithm
            # d_rot1 = atan2(self.dy, self.dx) - last_theta
            # d_trans = sqrt(self.dx**2 + self.dy**2)
            # d_rot2 = current_theta - last_theta - d_rot1

        # Now update positions of all particles

        for particle in self.particles:
            # # Lines 5-7 of the algorithm. I thinl + or - doesn't matter on the noise.
            # d_hat_rot1 = d_rot1 - random.gauss(0, 0.05)   # 0.25 std_dev
            # d_hat_trans = d_trans - random.gauss(0, 0.1) # 0.02
            # d_hat_rot2 = d_rot2 - random.gauss(0, 0.05)   # 0.25
            # # Lines 8-10
            # particle.x += d_hat_trans*cos(particle.theta + d_hat_rot1)
            # particle.y += d_hat_trans*sin(particle.theta + d_hat_rot1)
            # particle.theta += d_hat_rot1 + d_hat_rot2
            particle.x = particle.x + self.dx + random.gauss(0, 0.05)
            particle.y = particle.y + self.dy + random.gauss(0, 0.05)
            particle.theta = particle.theta + self.dyaw + random.gauss(0, 0.1)

        self.last_robot_odom = robot_odom
        """About the commented code: there is actually more than one way to implement the motion model.
        The Algorithm in the book has a simplified version. However, the straightforward way of doing the motion
        update is to calculate the dx,dx and dtheta in the particle's local frame, and add it to all the particles."""
    # End of algorithm

    # ----- Sensor Update Functions ----- #
    def calculate_mean_particle(self):
        # Calculate mean particle (before re-sampling)
        all_x = np.array([particle.x for particle in self.particles])
        all_y = np.array([particle.y for particle in self.particles])
        all_theta = np.array([particle.theta for particle in self.particles])

        weights = np.array(self.weights)

        x_mean = np.dot(weights, all_x)
        y_mean = np.dot(weights, all_y)
        theta_mean = np.dot(weights, all_theta)

        # Update pose estimate
        self.mean_estimate_particle.x = x_mean
        self.mean_estimate_particle.y = y_mean
        self.mean_estimate_particle.theta = theta_mean

    def handle_observation(self, laser_scan):
        """ Prediction and weight update"""
        errors = []
        print(len(self.particles))
        for particle in self.particles:
            error = self.get_prediction_error_squared(laser_scan, particle)
            errors.append(error)

        self.weights = [exp(-error) for error in errors]

        weight_sum = sum(self.weights)
        weights = np.array(self.weights)
        weights = weights/weight_sum
        self.weights = weights

    def get_prediction_error_squared(self, laser_scan_msg, particle):
        """
        This function evaluates the squared norm of the difference/error between the
        scan in laser_scan_msg and the one that was predicted by the given particle.
        """
        # Particles that are out of bounds of the map get a large error
        if particle.x < self.xmin or particle.x > self.xmax:
            return 1000

        if particle.y < self.ymin or particle.y > self.ymax:
            return 1000

        # If the particle falls inside an obstacle give it a large error
        row, col = self.metric_to_grid_coords(particle.x, particle.y)
        if row >= 201 or col >=201:
            return 1000

        if not self.grid_bin[row, col]:
            return 1000

        assert (self.laser_min_range >= 0)
        assert (self.laser_max_range > 0)

        # actual ranges and angles
        [actual_ranges, angles] = self.subsample_laser_scan(laser_scan_msg)
        self.ls_ranges = actual_ranges
        self.ls_angles = angles

        predict_ranges = self.simulate_laser_scan_for_particle(particle.x, particle.y, particle.theta, angles, self.laser_min_range, self.laser_max_range)

        diff = [actual_range - predict_range for actual_range, predict_range in zip(actual_ranges, predict_ranges)]

        # Take the squared norm of that difference
        norm_error = np.linalg.norm(diff)

        return norm_error

    def subsample_laser_scan(self, laser_scan_msg):
        """Subsamples a set number of beams (self.eval_beams) from the incoming actual laser scan. It also
        converts the Inf range measurements into max_range range measurements, in order to be able to
        compute a difference."""

        # To convert the laser points from the husky_1/base_laser frame, whose z-axis points downwards
        # to the same frame pointing upwards

        N = len(laser_scan_msg.ranges)
        ranges_in_upwards_baselaser_frame = laser_scan_msg.ranges
        angles_in_baselaser_frame = [(laser_scan_msg.angle_max - laser_scan_msg.angle_min)*float(i)/N + laser_scan_msg.angle_min for i in xrange(N)]

        step = N/self.eval_beams
        angles_in_upwards_baselaser_frame = angles_in_baselaser_frame[::step]
        ranges_in_upwards_baselaser_frame = ranges_in_upwards_baselaser_frame[::step]

        assert (len(ranges_in_upwards_baselaser_frame) == len(angles_in_upwards_baselaser_frame))

        actual_ranges = []
        for r in ranges_in_upwards_baselaser_frame:
            if r >= self.laser_min_range and r <= self.laser_max_range:
                actual_ranges.append(r)

            if r < self.laser_min_range:
                actual_ranges.append(self.laser_min_range)

            if r > self.laser_max_range:
                actual_ranges.append(self.laser_max_range)

        return actual_ranges, angles_in_upwards_baselaser_frame

    def simulate_laser_scan_for_particle(self, x, y, yaw_in_map, angles, min_range, max_range):
        """If the robot was at the given particle, what would its laser scan
        be (in the known map)? Returns the predicted laser ranges if a particle with state (x,y,yaw_in_map)
        is to scan along relative angles in angles."""
        # for every relative angle in angles
        # 1. The absolute angle based on the robot's orientation is computed
        # 2. Ray tracing from (x,y) along the abosulte angle using step size range_step is done
        #    (a) If the currently examined point is within the bounds of the workspace
        #        stop if it meets an obstacle or if it reaches max_range
        #    (b) If the currently examined point is outside the bounds of the workspace
        #        stop if it reaches max_range
        # 3. The computed collection of ranges corresponding to the given angles is returned

        ranges = []
        range_step = self.ogm.info.resolution

        for angle in angles:
            phi = yaw_in_map + angle

            r = min_range
            while r <= max_range:
                xm = x + r*cos(phi)
                ym = y + r*sin(phi)

                if xm > self.xmax or xm < self.xmin:
                    break

                if ym > self.ymax or ym < self.ymin:
                    break

                row, col = self.metric_to_grid_coords(xm, ym)
                free = self.grid_bin[row, col].all()
                if not free:
                    break

                r += range_step

            ranges.append(r)

        return ranges

