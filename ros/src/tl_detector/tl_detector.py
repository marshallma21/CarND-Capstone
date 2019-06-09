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

DEBUG_IMAGE_SWITCH = True

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector', log_level=rospy.DEBUG)
        rospy.loginfo("[tl_detector] Welcome to tl_detector")

        self.pose = None

        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.camera_image = None
        self.lights = []
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

        self.light_classifier = TLClassifier_YOLOv3(DEBUG_OUTPUT=DEBUG_IMAGE_SWITCH)
        #self.light_classifier = TLClassifier_SSD(DEBUG_OUTPUT=DEBUG_IMAGE_SWITCH)

        self.distance_to_tl_threshold = 67.0
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_stop_wp_idx = -1
        self.last_tl_idx = -1
        self.last_tl_wp_idx = -1
        self.state_count = 0
        self.last_wp = -1

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

        if DEBUG_IMAGE_SWITCH:
            self.DEBUG_IMG_pub = rospy.Publisher('/detector_image', Image, queue_size=1)

        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_lights_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        rospy.spin()

    def traffic_lights_cb(self, msg):
        self.lights = msg.lights

        # Dump the traffic lights, stop lines, and base waypoints
        if self.wl_debug_lights < 1:
            self.wl_debug_lights += 1
            if (self.config is not None):   
                rospy.logdebug("\n[tl_detector] %s traffic_lights_cb stoplines.len=%s ", 
                                self.wl_debug_lights, len(self.config['stop_line_positions']))
                for n in range(len(self.config['stop_line_positions'])):
                    rospy.logdebug("[tl_detector] stop_line_positions[%s] : x=%s, y=%s ", n, 
                                    self.config['stop_line_positions'][n][0],
                                    self.config['stop_line_positions'][n][1])
                                         
            if (self.lights is not None):       
                rospy.logdebug("\n[tl_detector] %s traffic_lights_cb self.lights.len=%s ", 
                            self.wl_debug_lights, len(self.lights))
                for n in range(len(self.lights)):
                    rospy.logdebug("[tl_detector] lights[%s] : [%s, %s], ", n, 
                                    self.lights[n].pose.pose.position.x,
                                    self.lights[n].pose.pose.position.y)
                                
            if (self.base_waypoints is not None) and (self.is_carla is True):
                rospy.logdebug("\n[tl_detector] %s traffic_lights_cb self.base_waypoints.len=%s ", 
                            self.wl_debug_lights, len(self.base_waypoints.waypoints))
                for n in range(len(self.base_waypoints.waypoints)):
                    rospy.logdebug("[tl_detector] base_wp[%s] : [%s, %s], ", n, 
                                    self.base_waypoints.waypoints[n].pose.pose.position.x,
                                    self.base_waypoints.waypoints[n].pose.pose.position.y)

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

    def dist_to_point(self, pose, wp_pose):
        x_squared = pow((pose.position.x - wp_pose.position.x), 2)
        y_squared = pow((pose.position.y - wp_pose.position.y), 2)
        dist = sqrt(x_squared + y_squared)
        return dist

    def get_closest_wp_idx(self, pose, waypoints):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
            waypoints : points where to look for closest one

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        min_dist = float("inf")
        closest_wp_idx = -1

        if not waypoints:
            rospy.logwarn("[TL_DETECTOR] No waypoints given.")
        else:
            for idx, wp in enumerate(waypoints):
                dist = self.dist_to_point(pose, wp.pose.pose)
                if(dist < min_dist):
                    min_dist = dist
                    closest_wp_idx = idx
        return closest_wp_idx

    def detect_tl(self):
        #rospy.loginfo("Detection start")
        
        stop_wp_idx, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        car_wp_idx = -1
        if (self.pose is not None) and (self.base_waypoints is not None): 
            car_wp_idx = self.get_closest_wp_idx(self.pose.pose, self.base_waypoints.waypoints)

        if self.state != state:
            rospy.logwarn("[detect_tl] Traffic light changed: last_stop_wp_idx:%s, car_wp_idx:%s,   %d, %s ---> %d, %s ",
                                    self.last_stop_wp_idx, car_wp_idx, 
                                    self.state, TRAFFIC_LIGHT_NAME[self.state], state, TRAFFIC_LIGHT_NAME[state])
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            stop_wp_idx = stop_wp_idx if state == TrafficLight.RED else -1
            self.last_stop_wp_idx = stop_wp_idx
            self.upcoming_red_light_pub.publish(Int32(self.last_stop_wp_idx))

            ego_go = "Stop!"
            if stop_wp_idx == -1:
                ego_go = "Go!"            
            rospy.logdebug("[detect_tl] %3d Pub traffic_waypoint.1 last_stop_wp:%s, car_wp:%s, state=%s, %s \n", 
                                            self.state_count, self.last_stop_wp_idx, car_wp_idx, self.state, ego_go)
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_stop_wp_idx))
            rospy.logdebug("[detect_tl] %3d Pub traffic_waypoint.2 last_stop_wp:%s, car_wp:%s. state=%s \n", 
                                            self.state_count, self.last_stop_wp_idx, car_wp_idx, self.state)

        self.state_count += 1

        self.thread_working = False

        if DEBUG_IMAGE_SWITCH:
            try:
                image_message = self.bridge.cv2_to_imgmsg(self.light_classifier.DEBUG_IMAGE, encoding="bgr8")   
                self.DEBUG_IMG_pub.publish(image_message)
            except:
                rospy.logwarn("Unable to get debug image")


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
        if (self.pose is not None) and (self.has_image) and (self.base_waypoints is not None):            
            tl_idx = self.get_closest_wp_idx(self.pose.pose, self.lights)                
            if tl_idx >= 0: 
                ########################################################################
                #  |(car position)           |(stop line)             |(traffic light) #
                #  |<---car_stop distance--->|<---stop_tl distance--->|                #
                ########################################################################                
                    
                # Nearest car position waypoint index
                car_wp_idx = self.get_closest_wp_idx(self.pose.pose, self.base_waypoints.waypoints)
                
                # Nearest stop line waypoint index
                stop_line = self.stop_line_positions[tl_idx]
                stop_line_pose = Pose()
                stop_line_pose.position.x = stop_line[0]
                stop_line_pose.position.y = stop_line[1]
                stop_wp_idx = self.get_closest_wp_idx(stop_line_pose, self.base_waypoints.waypoints)
                if (stop_wp_idx == -1):
                    rospy.logdebug("[TL] Unable to determine valid TL idx.")
                    return -1, TrafficLight.UNKNOWN
                    
                # Nearest traffic light waypoint index
                tl_wp = self.lights[tl_idx]
                tl_wp_idx = self.get_closest_wp_idx(tl_wp.pose.pose, self.base_waypoints.waypoints)

                # Distance from car position to stop line 
                car_stop_dist = self.dist_to_point(self.pose.pose, stop_line_pose)
                if (car_wp_idx > stop_wp_idx):
                    car_stop_dist = 0 - car_stop_dist
                    
                if (self.last_tl_idx != tl_idx):
                    rospy.logwarn("[TL] New traffic lights founded  idx: %d(wp:%d) ----> idx: %d(wp:%d)\n",
                                        self.last_tl_idx, self.last_tl_wp_idx, tl_idx, tl_wp_idx)
                    self.last_tl_idx = tl_idx
                    self.last_tl_wp_idx = tl_wp_idx
                    
                rospy.logdebug("[TL] Closest car_wp:%s, stop_wp:%s, tl_wp:%s.  car_stop_dist:%.2f",
                            car_wp_idx, stop_wp_idx, tl_wp_idx, car_stop_dist)
                
                state = TrafficLight.UNKNOWN
                  
                # Detected traffic light should be ahead of car position
                if (tl_wp_idx >= car_wp_idx): 
                    # Only detect traffic lights in the camera image in range of distance_to_tl_threshold
                    if (car_stop_dist < self.distance_to_tl_threshold):                    
                        start = time.time()
                        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8") 
                        self.number_of_detected_lights = self.light_classifier.detect_traffic_lights(cv_image)                        
                        end1 = time.time()
                        rospy.logdebug("[TL] Detection Time:%f s, num of lights %d", 
                                                end1 - start, self.number_of_detected_lights)
                                                
                        if self.number_of_detected_lights > 0:
                            state = self.light_classifier.get_classification()

                            rospy.logdebug("[TL] ------------------------------ light state %s, %s ", state, TRAFFIC_LIGHT_NAME[state]) 
                                                                                 
                        else:
                            rospy.logwarn("[TL] No trafic light found! %s", self.number_of_detected_lights)                                             
                    else:
                        rospy.logdebug("[TL] Next TL too far yet. car_stop_dist=%.2f", car_stop_dist)                                            
                else:
                    rospy.logdebug("[TL] Nearest TL passed. car_stop_dist= %.2f", car_stop_dist)
                
                
                # Distance from stop line to traffic light
                stop_wp = self.base_waypoints.waypoints[stop_wp_idx]
                stop_tl_dist = self.dist_to_point(tl_wp.pose.pose, stop_wp.pose.pose)
                
                rospy.logdebug("[TL] Car position(%s):(x,y)=(%.2f,%.2f); car_stop_dist=%.2f", 
                                car_wp_idx, self.pose.pose.position.x, self.pose.pose.position.y, car_stop_dist)
                rospy.logdebug("[TL] ClosestStopL(%s):(x,y)=(%.2f,%.2f); stop_tl_dist=%.2f", 
                                stop_wp_idx, stop_line[0], stop_line[1], stop_tl_dist)
                rospy.logdebug("[TL] TrafficLight(%s):(x,y)=(%.2f,%.2f); Upcoming tl_idx:%s", 
                                tl_wp_idx, tl_wp.pose.pose.position.x, tl_wp.pose.pose.position.y, tl_idx)                     

                return stop_wp_idx, state         
                        
            else:
                rospy.logwarn("[TL] No trafic stop line found!")
        else:
            rospy.logwarn("[TL] No EGO position available!")

        return -1, TrafficLight.UNKNOWN
     

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

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
