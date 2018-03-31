#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''

        # Queue size = 5 - 7 works best, but still has ~500ms latency
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=5)


        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

        ### Everything below is for testing purpose only while the classifier is not working
        light_wp, state = self.process_traffic_lights()


    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
        # rospy.logwarn("IMG")
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

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #Calculate the  distance between pose and first waypoint and set as min_dist to beat
        #Set waypoint that is the closesnt to traffic light as first waypoint
        min_dist = 0.0
        loc = -1;
        #Run through waypoints & perform distance calculation to find closest waypoint

        # rospy.logwarn(self.waypoints)
        # Read the documentation about waypoint (Lane) message 
        # Section 4 in Project description

        if self.waypoints is None:
            return

        
        for i, wp in enumerate(self.waypoints.waypoints):
            wpx = wp.pose.pose.position.x
            wpy = wp.pose.pose.position.y
            pose_x = pose.position.x
            pose_y = pose.position.y

            diff_x = pose_x - wpx
            diff_y = pose_y - wpy
            # diff_x_sq = pow(diff_x,2)
            # diff_y_sq = pow(diff_y,2)

            #if temp distance is closer than previous distance, then this is the new closest wp
            temp_dist = (diff_x**2 + diff_y**2) ** 0.5

            # Unfortunately, Python does not have sqrt() without import math
            # temp_dist = sqrt(diff_x_sq + diff_y_sq)
            if(loc == -1):
                #set new distance to beat
                min_dist = temp_dist
                #set new closest waypoint
                loc = i
            elif(temp_dist < min_dist):
                #set new distance to beat
                min_dist = temp_dist
                #set new closest waypoint
                loc = i

        #rospy.logwarn(loc)
        return loc

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        rospy.logwarn(light) # I will just leave this here so that we can test when the classifier is ready
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def get_faked_light_state(self, light):
        '''
        Get the ground truth light state from the simulator.

        Testing only. Not for production use. 
        
        Remove this method after we get a working classifier. Use get_light_state() instead

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        '''

        return light.state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light_idx = None
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

        # Find the closest visible traffic light (if one exists)
        # Identify what light is closest and in front of the car, based on waypoints
        closest_light_waypoint = -1  #initialize waypoint to behind the vehicle

        # Check the closest waypoint in this lap
        for l, lite in enumerate(self.lights):

            #Find closest waypoint for nearby light
            # TypeError: 'TrafficLight' object has no attribute '__getitem__'

            # I am pretty sure that you want to pass in the position of the traffic light
            temp_waypoint = self.get_closest_waypoint(lite.pose.pose)

            #Check if temp_waypoint is in front of the car
            if (temp_waypoint > car_position):
                #if a closest_light_waypoint hasn't been assigned already then assign
                if(closest_light_waypoint == -1):
                    closest_light_waypoint = temp_waypoint
                    # Assign properties of newly identified closest light to variable "light"
                    light_idx = l
                    light = lite
                #Otherwise a closest_light_waypoint has been assigned already, check if this one is closer 
                elif( temp_waypoint < closest_light_waypoint):
                    closest_light_waypoint =  temp_waypoint
                    # Assign properties of  even closer light to variable "light"
                    light_idx = l
                    light = lite

        # if we cannot find a waypoint, find it in next lap
        # In this case, the closest one must be the first one if we are driving in counter-clockwise
        # TODO: What happens if we drive clockwise?
        if not light:
            light_idx = 0
            light = self.lights[0]


        rospy.logwarn("{}, {}".format(light_idx, light.state))
        # if we have found a closest light to monitor, then determine the stop line position of this light
        if light:
            light_wp = self.get_closest_waypoint(stop_line_positions[light])
            state = self.get_light_state(light)

            state_2 = self.get_faked_light_state(light)
            # rospy.logwarn(state_2)
            return light_wp, state

        # Waypoint callback is only called once per program execution, you should not clear this for each timestep
        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
