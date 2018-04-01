#!/usr/bin/env python

import math

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, sys
from styx_msgs.msg import Lane

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

LOOKAHEAD_WPS = 50  # Number of waypoints we will publish.
MAX_ACCEL = 2.0


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=2)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=8)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=8)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=LOOKAHEAD_WPS)

        # TODO: Add other member variables you need below

        # Node-wide variables
        self.base_waypoints = None
        self.total_waypoints = 0

        # The index of the closest waypoint to the car
        self.closest_waypoint_idx = -1
        self.current_pos = None
        self.current_velocity = None


        self.max_speed_mps = self.kph2mps(rospy.get_param('~velocity'))

        # The following speed limit is for testing only
        # self.max_speed_mps = self.kph2mps(150)
        rospy.logdebug("Max speed={}mps".format(self.max_speed_mps))

        rospy.spin()

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

                # As soon as the first waypoint is found, populate LOOKAHEAD_WPS waypoints into the list sequentially
                for j in range(LOOKAHEAD_WPS):
                    j_mod = i + j % self.total_waypoints

                    next_wp = self.base_waypoints.waypoints[j_mod]
                    next_wp.twist.twist.linear.x = min((self.current_velocity + (j + 1) * MAX_ACCEL),
                                                       self.max_speed_mps)
                    rospy.logdebug("Next waypoint idx={}, velocity={}".format(j, next_wp.twist.twist.linear.x))

                    final_waypoints_list.append(next_wp)
                # Format the message
                msg = Lane()
                msg.waypoints = final_waypoints_list
                # Setting header info to current_pos header
                msg.header = self.current_pos.header
                self.final_waypoints_pub.publish(msg)
                return

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
            dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)
            dist = dl(waypoint_position, tl_position)
            if dist < closest_dist:
                waypoint_idx = i
                closest_dist = dist

        return waypoint_idx

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
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
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
