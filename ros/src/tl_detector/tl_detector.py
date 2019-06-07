#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import tf
import cv2
import yaml
from scipy.spatial import KDTree
import time
import thread

import numpy as np
from math import pow, sqrt

#from light_classification.tl_classifier_ssd import TLClassifier_SSD
from light_classification.tl_classifier_yolov3 import TLClassifier_YOLOv3
from light_classification.tl_classifier_ssd import TLClassifier_SSD

STATE_COUNT_THRESHOLD = 3

SMOOTH = 1.
TRAFFIC_LIGHT_NAME = ['RED','YELLOW','GREEN', 'Invalid', 'UNKNOWN']


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector', log_level=rospy.DEBUG)
        rospy.loginfo("[tl_detector] Welcome to tl_detector")

        self.pose = None

        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.camera_image = None
        #self.lights = []
        self.number_of_detected_lights = 0
        self.has_image = False
        self.thread_working = False

        self.frame_count = 0


        self.bridge = CvBridge()

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.is_carla = self.config['is_site']
        rospy.set_param('is_carla',self.is_carla)
        rospy.loginfo("[tl_detector] Is site running: %s", self.is_carla)

        #self.light_classifier = TLClassifier_YOLOv3()
        self.light_classifier = TLClassifier_SSD()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.wl_debug_lights = 0
        self.lights = []

        self.stop_line_positions = self.config['stop_line_positions']

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.DEBUG_IMG_pub = rospy.Publisher('/detector_image', Image, queue_size=1)
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_lights_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        rospy.spin()

    def traffic_lights_cb(self, msg):
        self.lights = msg.lights

        if self.wl_debug_lights < 1:
            self.wl_debug_lights += 1
            rospy.logdebug("[tl_detector] %s traffic_lights_cb stoplines.len=%s ", 
                            self.wl_debug_lights, len(self.config['stop_line_positions']))
            for n in range(len(self.config['stop_line_positions'])):
                rospy.logdebug("[tl_detector] stop_line_positions[%s] : x=%s, y=%s ", n, 
                                self.config['stop_line_positions'][n][0],
                                self.config['stop_line_positions'][n][1])
                                                
            rospy.logdebug("[tl_detector] %s traffic_lights_cb self.lights.len=%s ", 
                        self.wl_debug_lights, len(self.lights))
            for n in range(len(self.lights)):
                rospy.logdebug("[tl_detector] lights[%s] : [%s, %s], ", n, 
                                self.lights[n].pose.pose.position.x,
                                self.lights[n].pose.pose.position.y)
    
    def pose_cb(self, msg):
        self.pose = msg

    def base_waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        
        if not self.waypoints_2d:
            self.waypoints_2d = [[wp.pose.pose.position.x, wp.pose.pose.position.y] 
                                    for wp in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    '''
    def traffic_cb(self, msg):
        self.lights = msg.lights
    '''

    def detect_tl(self):
        #rospy.loginfo("Detection start")
        start = time.time()

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        self.number_of_detected_lights = self.light_classifier.detect_traffic_lights(cv_image)
        
        end = time.time()
        rospy.logdebug("[tl_detector] Detection Time:%f s", end - start)

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

        #image_message = self.bridge.cv2_to_imgmsg(self.light_classifier.DEBUG_IMAGE, encoding="bgr8")   
        #self.DEBUG_IMG_pub.publish(image_message)

        self.thread_working = False

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        #rospy.loginfo("Image_cb")
        self.has_image = True

        if not self.thread_working:
            self.thread_working = True
            self.camera_image = msg
            thread.start_new_thread( self.detect_tl, ())


    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        closest_idx = self.waypoint_tree.query([x,y], 1)[1]
        return closest_idx

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
       
        self.base_waypoints = None     int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """     
        rospy.logdebug("[tl_detector] Process_traffic_light %d", self.number_of_detected_lights)

        if (self.pose is not None) and (self.has_image) and (self.base_waypoints is not None):            
            closet_stop_line_wp_idx = -1
            state = None

            if self.number_of_detected_lights > 0:
                state = self.light_classifier.get_classification()
                if state != None:
                    rospy.logwarn("[tl_detector] Confirmed traffic light %s:", TRAFFIC_LIGHT_NAME[state])

                if state == TrafficLight.RED:
                    car_pose_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

                    smallest_wp_dist = len(self.base_waypoints.waypoints)
                    for i in range(len(self.stop_line_positions)):
                        stop_line_point = self.stop_line_positions[i]
                        stop_line_point_wp_idx = self.get_closest_waypoint(stop_line_point[0], stop_line_point[1])

                        dist = stop_line_point_wp_idx - car_pose_wp_idx
                        if dist >= 0 and dist < smallest_wp_dist:
                            smallest_wp_dist = dist
                            closet_stop_line_wp_idx = stop_line_point_wp_idx
                    rospy.logwarn("[tl_detector] Closet waypoint is %d:", closet_stop_line_wp_idx)        
                    return closet_stop_line_wp_idx, state


        if self.pose is None:
            rospy.logwarn("[tl_detector] No EGO position available!")
        if not self.has_image:
            rospy.logwarn("[tl_detector] No camera_image available!")

        

        #Simulation code start
        '''
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            #car_position = self.get_closest_waypoint(self.pose.pose)
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

            #TODO find the closest visible traffic light (if one exists)
            diff = len(self.base_waypoints.waypoints)
            for i in range(len(self.lights)):
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                d = temp_wp_idx - car_wp_idx
                if d >= 0 and d < diff:
                    diff = d
                    closest_light = self.lights[i]
                    line_wp_idx = temp_wp_idx
        '''
        #Simulation code end
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
