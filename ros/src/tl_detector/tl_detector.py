#!/usr/bin/env python
import time

import cv2
import rospy
import tf
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from light_classification.tl_classifier import TLClassifier
from scipy.spatial import KDTree
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, TrafficLightArray, TrafficLight

STATE_COUNT_THRESHOLD = 3
# Test mode uses "/vehicle/traffic_lightsTrue for Ground Truth Traffic Data
# False for Model Prediction Traffic Data
TEST_MODE_ENABLED = False
SAVE_IMAGES = False
LOGGING_THROTTLE_FACTOR = 5  # Only log at this rate (1 / Hz)


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.camera_image = None
        self.lights = []

        self.bridge = CvBridge()

        self.light_classifier = TLClassifier(rospy.get_param('~model_file'))
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.class_count = 0
        self.process_count = 0

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=2)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=8)
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=2)
        rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in
                                 waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """
        Identifies red lights in the incoming camera image and publishes the index of the waypoint closest to the red
        light's stop line to /traffic_waypoint
        :param msg: image from car-mounted camera
        """
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def process_traffic_lights(self):
        """
        Finds closest visible traffic light, if one exists, and determines its location and color
        :return:
        int: index of waypoint closest to the upcoming stop line for a traffic light (-1 if none exists)
        int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        closest_light = None
        line_wp_idx = -1
        state = TrafficLight.UNKNOWN

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if self.pose:
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

            diff = len(self.base_waypoints.waypoints)
            for i, light in enumerate(self.lights):
                # Get stop line waypoint index
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                # Find closest stop line waypoint index
                d = temp_wp_idx - car_wp_idx
                if 0 <= d < diff:
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx

        # if we have found a closest light to monitor, then determine the stop line position of this light
        if closest_light:
            self.process_count += 1
            state = self.get_light_state(closest_light)
            if (self.process_count % LOGGING_THROTTLE_FACTOR) == 0:
                rospy.logwarn("DETECT: line_wp_idx={}, state={}".format(line_wp_idx, self.to_string(state)))

        return line_wp_idx, state

    def get_closest_waypoint(self, x, y):
        """
        Identifies the closest path waypoint to the given position
        https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        :param x: x coord position to match a waypoint to
        :param y: y coord position to match a waypoint to
        :return: index of the closest waypoint in self.waypoints
        """
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        return closest_idx

    def get_light_state(self, light):
        """
        Determines the current color of the traffic light
        :param light: light to classify
        :return: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        # For test mode, just return the light state
        if TEST_MODE_ENABLED:
            classification = light.state
        else:
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

            # Get classification
            classification = self.light_classifier.get_classification(cv_image)

            # Save image (throttled)
            if SAVE_IMAGES and (self.process_count % LOGGING_THROTTLE_FACTOR == 0):
                save_file = "../../../imgs/{}-{:.0f}.jpeg".format(self.to_string(classification), (time.time() * 100))
                cv2.imwrite(save_file, cv_image)

        return classification

    def to_string(self, state):
        out = "unknown"
        if state == TrafficLight.GREEN:
            out = "green"
        elif state == TrafficLight.YELLOW:
            out = "yellow"
        elif state == TrafficLight.RED:
            out = "red"
        return out


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
