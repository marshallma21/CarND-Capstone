import rospy

import numpy as np
import cv2
import tensorflow as tf
from keras.models import load_model
from styx_msgs.msg import TrafficLightArray, TrafficLight


# Uncomment the following code if need to visualize the detection output
# os.chdir(cwd+'/models')
# from object_detection.utils import visualization_utils as vis_util

def plot_one_box(img, coord, label=None, score=None, color=None, line_thickness=None):
    '''
    coord: [x_min, y_min, x_max, y_max] format coordinates.
    img: img to plot on.
    label: str. The label name.
    color: int. color index.
    line_thickness: int. rectangle line thickness.
    '''
    tl = line_thickness or int(round(0.002 * max(img.shape[0:2])))  # line thickness
    color = color or [255,0,0]
    c1, c2 = (int(coord[0]), int(coord[1])), (int(coord[2]), int(coord[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl)
    if label:
        label = '%s:%.2f'%(label,score)
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=float(tl) / 5, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1)  # filled
        cv2.putText(img, label, (c1[0], c1[1] - 2), 0, float(tl) / 5, [0, 0, 0], thickness=tf, lineType=cv2.LINE_AA)

class TLClassifier_SSD(object):
    def __init__(self, DEBUG_OUTPUT=False):
        self.DEBUG_SWITCH = DEBUG_OUTPUT
        self.is_carla=rospy.get_param("is_carla")
        #self.is_carla=True
        self.threshold =rospy.get_param("~threshold")
        #self.threshold = 0.01
        self.hw_ratio = rospy.get_param("~hw_ratio")
        #self.hw_ratio = 0.5  # height_width ratio
        print('Initializing classifier with threshold =', self.threshold)
        self.signal_classes = ['Red', 'Green', 'Yellow']
        self.light_state = TrafficLight.UNKNOWN

        if self.DEBUG_SWITCH:
            self.DEBUG_IMAGE = None

        # if sim_testing, we use a detection and classification models
        # if site_testing, we use a single model which does both detection and classification
        model_path_base='models/ssd/'
        if not self.is_carla:  # we use different models for classification
            # keras classification model
            self.cls_model = load_model(model_path_base+'tl_model_5.h5')  # switched to model 5 for harsh light
            self.graph = tf.get_default_graph()
            # tensorflow localization/detection model
            PATH_TO_CKPT = 'frozen_inference_graph_ssd_mobilenet_v1_coco_11_06_2017.pb'
            # setup tensorflow graph
            self.detection_graph = tf.Graph()
            # configuration for possible GPU
            config = tf.ConfigProto()
            config.gpu_options.allow_growth = True

        else:  # site testing where we load the localization+classification model
            PATH_TO_CKPT = 'frozen_inference_graph_ssd_kyle_v4.pb'
            # setup tensorflow graph
            self.detection_graph = tf.Graph()

            # configuration for possible GPU use
            config = tf.ConfigProto()
            config.gpu_options.allow_growth = True
            # load frozen tensorflow detection model and initialize
            # the tensorflow graph

        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_path_base+PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            self.sess = tf.Session(graph=self.detection_graph, config=config)
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            # Each box represents a part of the image where a particular object was detected.
            self.boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            self.scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')



    # Helper function to convert normalized box coordinates to pixels
    def box_normal_to_pixel(self, box, dim):

        height, width = dim[0], dim[1]
        box_pixel = [int(box[0] * height), int(box[1] * width), int(box[2] * height), int(box[3] * width)]
        return np.array(box_pixel)

    def get_simulator_localization(self, image):
        """Determines the locations of the traffic light in the image

        Args:
            image: camera image

        Returns:
            list of integers: coordinates [x_left, y_up, x_right, y_down]

        """

        with self.detection_graph.as_default():
            image_expanded = np.expand_dims(image, axis=0)
            (boxes, scores, classes, num_detections) = self.sess.run(
                [self.boxes, self.scores, self.classes, self.num_detections],
                feed_dict={self.image_tensor: image_expanded})

            boxes = np.squeeze(boxes)
            classes = np.squeeze(classes)
            scores = np.squeeze(scores)

            
            if self.DEBUG_SWITCH:
                height_ori, width_ori = image.shape[:2]
                for i in range(len(boxes)):
                    if scores[i] > 0.15:
                        x0, y0, x1, y1 = boxes[i]
                        x0 = max(0, int(x0*height_ori))
                        x1 = min(height_ori, int(x1*height_ori))
                        y0 = max(0, int(y0*width_ori))
                        y1 = min(width_ori, int(y1*width_ori))

                        plot_one_box(image, [y0, x0, y1, x1])

                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                self.DEBUG_IMAGE = image

            cls = classes.tolist()
            # print(cls)
            # Find the first occurence of traffic light detection id=10
            idx = next((i for i, v in enumerate(cls) if v == 10.), None)
            # If there is no detection
            if idx == None:
                box = [0, 0, 0, 0]
                print('no detection!')
            # If the confidence of detection is too slow, 0.3 for simulator
            elif scores[idx] <= self.threshold:  # updated site treshold to 0.01 for harsh light
                box = [0, 0, 0, 0]
                print('low confidence:', scores[idx])
            # If there is a detection and its confidence is high enough
            else:
                # *************corner cases***********************************
                dim = image.shape[0:2]
                box = self.box_normal_to_pixel(boxes[idx], dim)
                box_h = box[2] - box[0]
                box_w = box[3] - box[1]
                ratio = box_h / (box_w + 0.01)
                # if the box is too small, 20 pixels for simulator
                if (box_h < 20) or (box_w < 20):
                    box = [0, 0, 0, 0]
                    print('box too small!', box_h, box_w)
                # if the h-w ratio is not right, 1.5 for simulator, 0.5 for site
                elif (ratio < self.hw_ratio):
                    box = [0, 0, 0, 0]
                    print('wrong h-w ratio', ratio)
                else:
                    print(box)
                    print('localization confidence: ', scores[idx])
                    rospy.logdebug('localization confidence:==============================='+str(scores[idx]))
            # ****************end of corner cases***********************

        return box

    def get_simulator_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): cropped image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        signal_status = TrafficLight.UNKNOWN
        img_resize = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # Convert to four-dimension input as required by Keras
        img_resize = np.expand_dims(img_resize, axis=0).astype('float32')
        # Normalization
        img_resize /= 255.
        # Prediction
        with self.graph.as_default():
            predict = self.cls_model.predict(img_resize)
            # Get color classification
            tl_color = self.signal_classes[np.argmax(predict)]
            # TrafficLight message
        # uncomment the following in real test
        if tl_color == 'Red':
            signal_status = TrafficLight.RED
        elif tl_color == 'Green':
            signal_status = TrafficLight.GREEN
        elif tl_color == 'Yellow':
            signal_status =  TrafficLight.YELLOW

        return signal_status

    # Main function for end-to-end bounding box localization and light color
    # classification
    def get_carla_localization_classification(self, image):
        """Determines the locations of the traffic light in the image

        Args:
            image: camera image

        Returns:
            box: list of integer for coordinates [x_left, y_up, x_right, y_down]
            conf: confidence
            cls_idx: 1->Green, 2->Red, 3->Yellow

        """

        with self.detection_graph.as_default():
            image_expanded = np.expand_dims(image, axis=0)
            (boxes, scores, classes, num_detections) = self.sess.run(
                [self.boxes, self.scores, self.classes, self.num_detections],
                feed_dict={self.image_tensor: image_expanded})

            boxes = np.squeeze(boxes)  # bounding boxes
            classes = np.squeeze(classes)  # classes
            scores = np.squeeze(scores)  # confidence

            
            if self.DEBUG_SWITCH:
                height_ori, width_ori = image.shape[:2]
                for i in range(len(boxes)):
                    if scores[i] > 0.15:
                        x0, y0, x1, y1 = boxes[i]
                        x0 = max(0, int(x0*height_ori))
                        x1 = min(height_ori, int(x1*height_ori))
                        y0 = max(0, int(y0*width_ori))
                        y1 = min(width_ori, int(y1*width_ori))

                        plot_one_box(image, [y0, x0, y1, x1])

                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                self.DEBUG_IMAGE = image

            cls = classes.tolist()

            # Find the most confident detection/classification
            idx = 0
            conf = scores[idx]
            cls_idx = cls[idx]

            # If there is no detection
            if idx == None:
                box = [0, 0, 0, 0]
                print('no detection!')
                cls_idx = 4.0
            # If the confidence of detection is too slow, 0.3 for simulator

            # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            elif scores[idx] <= 0.15:
                # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                box = [0, 0, 0, 0]
                print('low confidence:', scores[idx])
                cls_idx = 4.0
            # If there is a detection and its confidence is high enough
            else:
                # *************corner cases***********************************
                dim = image.shape[0:2]
                box = self.box_normal_to_pixel(boxes[idx], dim)
                box_h = box[2] - box[0]
                box_w = box[3] - box[1]
                ratio = box_h / (box_w + 0.01)
                # if the box is too small, 20 pixels for simulator
                if (box_h < 10) or (box_w < 10):
                    box = [0, 0, 0, 0]
                    cls_idx = 4.0
                    print('box too small!', box_h, box_w)
                # if the h-w ratio is not right, 1.5 for simulator
                # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                elif (ratio < 1.0):
                    # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    box = [0, 0, 0, 0]
                    cls_idx = 4.0
                    print('wrong h-w ratio', ratio)
                else:
                    print(box)
                    print('localization confidence: ', scores[idx])
                    rospy.logdebug('localization confidence:==============================='+str(scores[idx]))
            # ****************end of corner cases***********************

        return box, conf, cls_idx

    def detect_traffic_lights(self, cv_image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light_state = TrafficLight.UNKNOWN
        #TODO implement light color prediction
        if self.is_carla:
           processed_img = cv_image[0:600, 0:800]  # was [20:400, 0:800]
           processed_img = cv2.cvtColor(processed_img, cv2.COLOR_BGR2RGB)
           img_full_np=np.asarray(processed_img, dtype="uint8")
           b, conf, cls_idx = self.get_carla_localization_classification(img_full_np)
           print("Get out of Localization-Classification")
           if np.array_equal(b, np.zeros(4)):
               print('unknown')
               light_state = TrafficLight.UNKNOWN
           else:
               # light_state = cls_idx
               if cls_idx == 1.0:
                   print('Green', b)
                   light_state = TrafficLight.GREEN
               elif cls_idx == 2.0:
                   print('Red', b)
                   light_state = TrafficLight.RED
               elif cls_idx == 3.0:
                   print('Yellow', b)
                   light_state = TrafficLight.YELLOW
               elif cls_idx == 4.0:
                   print('Unknown', b)
                   light_state = TrafficLight.UNKNOWN
               else:
                   print('Really Unknown! Didn\'t process image well', b)
                   light_state = TrafficLight.UNKNOWN
        else:
            width, height, _ = cv_image.shape
            x_start = int(width * 0.10)
            x_end = int(width * 0.90)
            y_start = 0
            y_end = int(height * 0.85)
            processed_img = cv_image[y_start:y_end, x_start:x_end]
            processed_img = cv2.cvtColor(processed_img, cv2.COLOR_BGR2RGB)
            img_full_np = np.asarray(processed_img, dtype="uint8")
            b = self.get_simulator_localization(img_full_np)
            print(b)
            # If there is no detection or low-confidence detection
            if np.array_equal(b, np.zeros(4)):
                print('unknown')
                light_state = TrafficLight.UNKNOWN
            else:  # we can use the classifier to classify the state of the traffic light
                img_np = cv2.resize(processed_img[b[0]:b[2], b[1]:b[3]], (32, 32))
                light_state=self.get_simulator_classification(img_np)
        rospy.logdebug('light_state==============================='+str(light_state))

        self.light_state = light_state
        if light_state == TrafficLight.UNKNOWN:
            return 0
        else:
            return 1

    def get_classification(self):
        return self.light_state

if __name__ == '__main__':
    tl_cls=TLClassifier()
    img_path = ''
    cv_image = cv2.imread(img_path)
    cv_image = cv2.resize(cv_image, (800, 600))
    cv2.imshow('display', cv_image)
    cv2.waitKey()
    light_state = tl_cls.get_classification(cv_image)
