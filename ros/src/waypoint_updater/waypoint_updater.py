#!/usr/bin/env python

import math
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, sys
from scipy.spatial import KDTree
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray, TrafficLight

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish.
STOP_LIGHT_MARGIN = 30  # Distance in waypoints between the stop line and the stop light
STOP_LINE_MARGIN = 3.0  # Distance in meters to pad in front of the stop line
MIN_BRAKING_DIST = STOP_LIGHT_MARGIN * -0.5  # Keep braking for the stop light through this distance
# TODO: The SAFE_BRAKING_DIST should be derived on dist_to_stop_line AND current_velocity
SAFE_BRAKING_DIST = STOP_LIGHT_MARGIN * 2.5  # Distance to start braking for a stop light
MAX_ACCEL = 2.0
MAX_DECEL = 3.0
COAST_VELOCITY = 4.0  # Minimum velocity while coasting up to the light
LOGGING_THROTTLE_FACTOR = 10  # Only log after this many loops

# Test mode uses "/vehicle/traffic_lightsTrue for Ground Truth Traffic Data
# False for Model Prediction Traffic Data
TEST_MODE_ENABLED = True


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=2)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=8)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=8)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=2)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=LOOKAHEAD_WPS)

        # TODO: Add other member variables you need below

        # Node-wide variables
        self.pose = None
        self.base_waypoints = None
        self.total_waypoints = 0
        self.waypoints_2d = None
        self.waypoint_tree = None

        # A boolean to indicate that there is a red stop light ahead
        self.red_stop_light_ahead = True
        # The index of the waypoint of the red stop light
        self.red_stop_light_waypoint_idx = -1

        # The index of the closest waypoint to the car
        self.closest_waypoint_idx = -1
        self.current_pos = None
        self.current_velocity = None
        self.max_speed_mps = self.kph2mps(rospy.get_param('~velocity'))
        rospy.loginfo("Max speed={}, TEST_MODE_ENABLED={}".format(self.max_speed_mps, TEST_MODE_ENABLED))

        # rospy.spin()
        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # Could go all the way down to 30
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                #Get closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self, closest_idx):
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoint]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2) + pow((a.z - b.z), 2))
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x

    def convert_local(self, waypoint, current_pos):
        """
        Helper function for converting from global to local coordinates
        :param waypoint:
        :param current_pos:
        :return:
        """
        # Grab the waypoint and car position variables
        x_way = waypoint.pose.pose.position.x
        y_way = waypoint.pose.pose.position.y
        x_car = current_pos.pose.position.x
        y_car = current_pos.pose.position.y
        # Convert from Quarternion to Radians
        theta_car = 2 * math.acos(current_pos.pose.orientation.w)
        theta_waypoint = 2 * math.acos(waypoint.pose.pose.orientation.w)

        # Perform coordinate localization
        x_shift = x_way - x_car
        y_shift = y_way - y_car
        x = x_shift * math.cos(0 - theta_car) - y_shift * math.sin(0 - theta_car)
        y = x_shift * math.sin(0 - theta_car) + y_shift * math.cos(0 - theta_car)
        return x, y, theta_car, theta_waypoint

    def kph2mps(self, velocity_kph):
        return (velocity_kph * 1000.) / (60. * 60.)

    def pose_cb(self, msg):
        """
        Callback for the current car position received
        :param msg:
        :return:
        """
        self.current_pos = msg
        # x = msg.pose.position.x    #Access x value like this
        # theta = msg.pose.orientation.w # Access w value like this

        if self.base_waypoints is None or self.current_velocity is None:
            return

        # Create list for the published final_waypoints
        final_waypoints_list = []

        # Scan through all base_waypoints
        for i in range(len(self.base_waypoints.waypoints)):
            waypoint = self.base_waypoints.waypoints[i]

            # Convert waypoint to local coordinates
            x, y, theta_car, theta_waypoint = self.convert_local(waypoint, self.current_pos)
            orientation_match = math.cos(theta_waypoint - theta_car)

            # Check if the waypoint is in front of the car, and if the orientation is within +/- pi/4 of the car
            if x >= 0.00 and orientation_match > 0.707:
                self.closest_waypoint_idx = i

                if self.red_stop_light_ahead:
                    dist_to_stop_line = self.distance(self.base_waypoints.waypoints, self.closest_waypoint_idx,
                                                      self.red_stop_light_waypoint_idx) - STOP_LIGHT_MARGIN
                    # deceleration = (final_v^2 - curr_v^2) / (2 * distance)
                    decel = -1 * min((pow(self.current_velocity, 2) / (2 * dist_to_stop_line)), MAX_DECEL)
                else:
                    dist_to_stop_line = 0.0
                    decel = 0.0

                # Track first and last target velocities for logging
                initial_target_velocity = 0.0
                target_velocity = 0.0

                # As soon as the first waypoint is found, populate LOOKAHEAD_WPS waypoints into the list sequentially
                for j in range(LOOKAHEAD_WPS):
                    j_mod = i + j % self.total_waypoints

                    next_wp = self.base_waypoints.waypoints[j_mod]

                    target_velocity = self.calculate_target_velocity(decel, dist_to_stop_line, j)
                    if j == 0:
                        initial_target_velocity = target_velocity
                    next_wp.twist.twist.linear.x = target_velocity
                    final_waypoints_list.append(next_wp)

                # Throttle the logging a bit
                if (i % LOGGING_THROTTLE_FACTOR) == 0:
                    if self.red_stop_light_ahead:
                        rospy.logdebug(
                            "Stop light idx={}, closest idx={}, diff={}, dist_to_stop_line={:.2f}, decel={:.2f}".format(
                                self.red_stop_light_waypoint_idx, self.closest_waypoint_idx,
                                self.red_stop_light_waypoint_idx - self.closest_waypoint_idx, dist_to_stop_line, decel))

                    rospy.logdebug(
                        "Current velocity={:.2f}, target velocity[0]={:.2f}, target velocity[{}]={:.2f}".format(
                            self.current_velocity, initial_target_velocity, LOOKAHEAD_WPS, target_velocity))

                # Format the message
                msg = Lane()
                msg.waypoints = final_waypoints_list
                # Setting header info to current_pos header
                msg.header = self.current_pos.header
                self.final_waypoints_pub.publish(msg)
                return

    def calculate_target_velocity(self, decel, dist_to_stop_line, j):
        """
        Calculates velocity for a given waypoint based on the distance to the stop line and the calculated deceleration.
        :param decel: static deceleration value to use for the entire set of waypoint trajectories
        :param dist_to_stop_line: distance in meters to the stop line (includes STOP_LIGHT_MARGIN)
        :param j: index (out of LOOKAHEAD_WPS)
        :return: velocity (in m/s)
        """
        if self.red_stop_light_ahead and (MIN_BRAKING_DIST < dist_to_stop_line < SAFE_BRAKING_DIST):
            coast_velocity = (COAST_VELOCITY, 0.0)[dist_to_stop_line < STOP_LINE_MARGIN]
            calculated_velocity = math.sqrt(-2.0 * decel * dist_to_stop_line) - (j * 0.1)
            target_velocity = max(coast_velocity, min(calculated_velocity, self.max_speed_mps))
        else:
            target_velocity = min((self.current_velocity + (j + 1) * MAX_ACCEL), self.max_speed_mps)
        if target_velocity < 0.1:
            target_velocity = 0.0
        return target_velocity

    def waypoints_cb(self, waypoints):
        """
        Callback for base_waypoints when the simulator is started
        :param self:
        :param waypoints:
        :return:
        """

        # Set a variable for accessing base_waypoints throughout this node
        self.base_waypoints = waypoints
        # Set a variable for access the total number of base_waypoints throughout this node
        self.total_waypoints = len(self.base_waypoints.waypoints)
        rospy.logdebug("Total waypoints={}".format(self.total_waypoints))

    def get_closest_waypoint_idx(self, start_idx, pose):
        """
        Find the index of the traffic light starting from start_idx
        :param self:
        :param start_idx:
        :param pose:
        :return:
        """
        waypoint_idx = -1
        closest_dist = sys.maxint

        for i in range(self.total_waypoints):
            waypoint_position = self.base_waypoints.waypoints[(i + start_idx) % self.total_waypoints].pose.pose.position
            tl_position = pose.pose.position
            dl = lambda a, b: math.sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2))
            dist = dl(waypoint_position, tl_position)
            if dist < closest_dist:
                waypoint_idx = i
                closest_dist = dist

        return waypoint_idx



    def traffic_cb(self, msg):
        if self.current_velocity is None or self.current_pos is None or self.base_waypoints is None:
            return

        # For testing purpose we are subscribing to the /vehicle/traffic_lights
        traffic_lights = msg.lights
        closest_tl = None
        closest_dist = sys.maxint  # Some big number

        if TEST_MODE_ENABLED:
            for i in range(len(traffic_lights)):
                tl = msg.lights[i]
                if tl.state == TrafficLight.RED or tl.state == TrafficLight.YELLOW:  # Red or Yellow Traffic light
                    # Convert traffic light location to car local coordinates
                    x, y, theta_car, theta_tl = self.convert_local(tl, self.current_pos)
                    dl = lambda a, b: math.sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2))
                    dist = dl(tl.pose.pose.position,
                              self.base_waypoints.waypoints[self.closest_waypoint_idx].pose.pose.position)
                    if x > 0.0 and 0 < dist < closest_dist:
                        closest_tl = tl
                        closest_dist = dist

            if closest_tl is not None:
                tl_waypoint_idx = self.get_closest_waypoint_idx(0, closest_tl.pose)
                self.red_stop_light_waypoint_idx = tl_waypoint_idx
                self.red_stop_light_ahead = True
            else:
                self.red_stop_light_ahead = False

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
