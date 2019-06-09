from __future__ import division, print_function

import rospy
from sensor_msgs.msg import Image as ImageMsg
from styx_msgs.msg import TrafficLight

#import matplotlib.pyplot as plt

import glob
import os
import cv2
import numpy as np
import time
import tensorflow as tf
slim = tf.contrib.slim

from yolo_utils.misc_utils import parse_anchors, read_class_names
from yolo_utils.nms_utils import gpu_nms
from yolo_utils.plot_utils import get_color_table, plot_one_box
from yolo_utils.layer_utils import conv2d, darknet53_body, yolo_block, upsample_layer, LeakyRelu


class yolov3(object):

    def __init__(self, class_num, anchors, use_label_smooth=False, use_focal_loss=False, batch_norm_decay=0.999, weight_decay=5e-4):

        # self.anchors = [[10, 13], [16, 30], [33, 23],
                         # [30, 61], [62, 45], [59,  119],
                         # [116, 90], [156, 198], [373,326]]
        self.class_num = class_num
        self.anchors = anchors
        self.batch_norm_decay = batch_norm_decay
        self.use_label_smooth = use_label_smooth
        self.use_focal_loss = use_focal_loss
        self.weight_decay = weight_decay

    def forward(self, inputs, is_training=False, reuse=False):
        # the input img_size, form: [height, weight]
        # it will be used later
        self.img_size = tf.shape(inputs)[1:3]
        # set batch norm params
        batch_norm_params = {
            'decay': self.batch_norm_decay,
            'epsilon': 1e-05,
            'scale': True,
            'is_training': is_training,
            'fused': None,  # Use fused batch norm if possible.
        }

        with slim.arg_scope([slim.conv2d, slim.batch_norm], reuse=reuse):
            with slim.arg_scope([slim.conv2d], 
                                normalizer_fn=slim.batch_norm,
                                normalizer_params=batch_norm_params,
                                biases_initializer=None,
                                activation_fn=lambda x: LeakyRelu(x, alpha=0.1),
                                weights_regularizer=slim.l2_regularizer(self.weight_decay)):
                with tf.variable_scope('darknet53_body'):
                    route_1, route_2, route_3 = darknet53_body(inputs)

                with tf.variable_scope('yolov3_head'):
                    inter1, net = yolo_block(route_3, 512)
                    feature_map_1 = slim.conv2d(net, 3 * (5 + self.class_num), 1,
                                                stride=1, normalizer_fn=None,
                                                activation_fn=None, biases_initializer=tf.zeros_initializer())
                    feature_map_1 = tf.identity(feature_map_1, name='feature_map_1')

                    inter1 = conv2d(inter1, 256, 1)
                    inter1 = upsample_layer(inter1, tf.shape(route_2))
                    concat1 = tf.concat([inter1, route_2], axis=3)

                    inter2, net = yolo_block(concat1, 256)
                    feature_map_2 = slim.conv2d(net, 3 * (5 + self.class_num), 1,
                                                stride=1, normalizer_fn=None,
                                                activation_fn=None, biases_initializer=tf.zeros_initializer())
                    feature_map_2 = tf.identity(feature_map_2, name='feature_map_2')

                    inter2 = conv2d(inter2, 128, 1)
                    inter2 = upsample_layer(inter2, tf.shape(route_1))
                    concat2 = tf.concat([inter2, route_1], axis=3)

                    _, feature_map_3 = yolo_block(concat2, 128)
                    feature_map_3 = slim.conv2d(feature_map_3, 3 * (5 + self.class_num), 1,
                                                stride=1, normalizer_fn=None,
                                                activation_fn=None, biases_initializer=tf.zeros_initializer())
                    feature_map_3 = tf.identity(feature_map_3, name='feature_map_3')

            return feature_map_1, feature_map_2, feature_map_3

    def reorg_layer(self, feature_map, anchors):
        '''
        feature_map: a feature_map from [feature_map_1, feature_map_2, feature_map_3] returned
            from `forward` function
        anchors: shape: [3, 2]
        '''
        # NOTE: size in [h, w] format! don't get messed up!
        grid_size = tf.shape(feature_map)[1:3]  # [13, 13]
        # the downscale ratio in height and weight
        ratio = tf.cast(self.img_size / grid_size, tf.float32)
        # rescale the anchors to the feature_map
        # NOTE: the anchor is in [w, h] format!
        rescaled_anchors = [(anchor[0] / ratio[1], anchor[1] / ratio[0]) for anchor in anchors]

        feature_map = tf.reshape(feature_map, [-1, grid_size[0], grid_size[1], 3, 5 + self.class_num])

        # split the feature_map along the last dimension
        # shape info: take 416x416 input image and the 13*13 feature_map for example:
        # box_centers: [N, 13, 13, 3, 2] last_dimension: [center_x, center_y]
        # box_sizes: [N, 13, 13, 3, 2] last_dimension: [width, height]
        # conf_logits: [N, 13, 13, 3, 1]
        # prob_logits: [N, 13, 13, 3, class_num]
        box_centers, box_sizes, conf_logits, prob_logits = tf.split(feature_map, [2, 2, 1, self.class_num], axis=-1)
        box_centers = tf.nn.sigmoid(box_centers)

        # use some broadcast tricks to get the mesh coordinates
        grid_x = tf.range(grid_size[1], dtype=tf.int32)
        grid_y = tf.range(grid_size[0], dtype=tf.int32)
        grid_x, grid_y = tf.meshgrid(grid_x, grid_y)
        x_offset = tf.reshape(grid_x, (-1, 1))
        y_offset = tf.reshape(grid_y, (-1, 1))
        x_y_offset = tf.concat([x_offset, y_offset], axis=-1)
        # shape: [13, 13, 1, 2]
        x_y_offset = tf.cast(tf.reshape(x_y_offset, [grid_size[0], grid_size[1], 1, 2]), tf.float32)

        # get the absolute box coordinates on the feature_map 
        box_centers = box_centers + x_y_offset
        # rescale to the original image scale
        box_centers = box_centers * ratio[::-1]

        # avoid getting possible nan value with tf.clip_by_value
        box_sizes = tf.exp(box_sizes) * rescaled_anchors
        # box_sizes = tf.clip_by_value(tf.exp(box_sizes), 1e-9, 100) * rescaled_anchors
        # rescale to the original image scale
        box_sizes = box_sizes * ratio[::-1]

        # shape: [N, 13, 13, 3, 4]
        # last dimension: (center_x, center_y, w, h)
        boxes = tf.concat([box_centers, box_sizes], axis=-1)

        # shape:
        # x_y_offset: [13, 13, 1, 2]
        # boxes: [N, 13, 13, 3, 4], rescaled to the original image scale
        # conf_logits: [N, 13, 13, 3, 1]
        # prob_logits: [N, 13, 13, 3, class_num]
        return x_y_offset, boxes, conf_logits, prob_logits


    def predict(self, feature_maps):
        '''
        Receive the returned feature_maps from `forward` function,
        the produce the output predictions at the test stage.
        '''
        feature_map_1, feature_map_2, feature_map_3 = feature_maps

        feature_map_anchors = [(feature_map_1, self.anchors[6:9]),
                               (feature_map_2, self.anchors[3:6]),
                               (feature_map_3, self.anchors[0:3])]
        reorg_results = [self.reorg_layer(feature_map, anchors) for (feature_map, anchors) in feature_map_anchors]

        def _reshape(result):
            x_y_offset, boxes, conf_logits, prob_logits = result
            grid_size = tf.shape(x_y_offset)[:2]
            boxes = tf.reshape(boxes, [-1, grid_size[0] * grid_size[1] * 3, 4])
            conf_logits = tf.reshape(conf_logits, [-1, grid_size[0] * grid_size[1] * 3, 1])
            prob_logits = tf.reshape(prob_logits, [-1, grid_size[0] * grid_size[1] * 3, self.class_num])
            # shape: (take 416*416 input image and feature_map_1 for example)
            # boxes: [N, 13*13*3, 4]
            # conf_logits: [N, 13*13*3, 1]
            # prob_logits: [N, 13*13*3, class_num]
            return boxes, conf_logits, prob_logits

        boxes_list, confs_list, probs_list = [], [], []
        for result in reorg_results:
            boxes, conf_logits, prob_logits = _reshape(result)
            confs = tf.sigmoid(conf_logits)
            probs = tf.sigmoid(prob_logits)
            boxes_list.append(boxes)
            confs_list.append(confs)
            probs_list.append(probs)
        
        # collect results on three scales
        # take 416*416 input image for example:
        # shape: [N, (13*13+26*26+52*52)*3, 4]
        boxes = tf.concat(boxes_list, axis=1)
        # shape: [N, (13*13+26*26+52*52)*3, 1]
        confs = tf.concat(confs_list, axis=1)
        # shape: [N, (13*13+26*26+52*52)*3, class_num]
        probs = tf.concat(probs_list, axis=1)

        center_x, center_y, width, height = tf.split(boxes, [1, 1, 1, 1], axis=-1)
        x_min = center_x - width / 2
        y_min = center_y - height / 2
        x_max = center_x + width / 2
        y_max = center_y + height / 2

        boxes = tf.concat([x_min, y_min, x_max, y_max], axis=-1)

        return boxes, confs, probs
    
    def loss_layer(self, feature_map_i, y_true, anchors):
        '''
        calc loss function from a certain scale
        input:
            feature_map_i: feature maps of a certain scale. shape: [N, 13, 13, 3*(5 + num_class)] etc.
            y_true: y_ture from a certain scale. shape: [N, 13, 13, 3, 5 + num_class + 1] etc.
            anchors: shape [9, 2]
        '''
        
        # size in [h, w] format! don't get messed up!
        grid_size = tf.shape(feature_map_i)[1:3]
        # the downscale ratio in height and weight
        ratio = tf.cast(self.img_size / grid_size, tf.float32)
        # N: batch_size
        N = tf.cast(tf.shape(feature_map_i)[0], tf.float32)

        x_y_offset, pred_boxes, pred_conf_logits, pred_prob_logits = self.reorg_layer(feature_map_i, anchors)

        ###########
        # get mask
        ###########
        # shape: take 416x416 input image and 13*13 feature_map for example:
        # [N, 13, 13, 3, 1]
        object_mask = y_true[..., 4:5]
        # shape: [N, 13, 13, 3, 4] & [N, 13, 13, 3] ==> [V, 4]
        # V: num of true gt box
        valid_true_boxes = tf.boolean_mask(y_true[..., 0:4], tf.cast(object_mask[..., 0], 'bool'))

        # shape: [V, 2]
        valid_true_box_xy = valid_true_boxes[:, 0:2]
        valid_true_box_wh = valid_true_boxes[:, 2:4]
        # shape: [N, 13, 13, 3, 2]
        pred_box_xy = pred_boxes[..., 0:2]
        pred_box_wh = pred_boxes[..., 2:4]

        # calc iou
        # shape: [N, 13, 13, 3, V]
        iou = self.broadcast_iou(valid_true_box_xy, valid_true_box_wh, pred_box_xy, pred_box_wh)

        # shape: [N, 13, 13, 3]
        best_iou = tf.reduce_max(iou, axis=-1)

        # get_ignore_mask
        ignore_mask = tf.cast(best_iou < 0.5, tf.float32)
        # shape: [N, 13, 13, 3, 1]
        ignore_mask = tf.expand_dims(ignore_mask, -1)

        # get xy coordinates in one cell from the feature_map
        # numerical range: 0 ~ 1
        # shape: [N, 13, 13, 3, 2]
        true_xy = y_true[..., 0:2] / ratio[::-1] - x_y_offset
        pred_xy = pred_box_xy / ratio[::-1] - x_y_offset

        # get_tw_th
        # numerical range: 0 ~ 1
        # shape: [N, 13, 13, 3, 2]
        true_tw_th = y_true[..., 2:4] / anchors
        pred_tw_th = pred_box_wh / anchors
        # for numerical stability
        true_tw_th = tf.where(condition=tf.equal(true_tw_th, 0),
                              x=tf.ones_like(true_tw_th), y=true_tw_th)
        pred_tw_th = tf.where(condition=tf.equal(pred_tw_th, 0),
                              x=tf.ones_like(pred_tw_th), y=pred_tw_th)
        true_tw_th = tf.log(tf.clip_by_value(true_tw_th, 1e-9, 1e9))
        pred_tw_th = tf.log(tf.clip_by_value(pred_tw_th, 1e-9, 1e9))

        # box size punishment: 
        # box with smaller area has bigger weight. This is taken from the yolo darknet C source code.
        # shape: [N, 13, 13, 3, 1]
        box_loss_scale = 2. - (y_true[..., 2:3] / tf.cast(self.img_size[1], tf.float32)) * (y_true[..., 3:4] / tf.cast(self.img_size[0], tf.float32))

        ############
        # loss_part
        ############
        # mix_up weight
        # [N, 13, 13, 3, 1]
        mix_w = y_true[..., -1:]
        # shape: [N, 13, 13, 3, 1]
        xy_loss = tf.reduce_sum(tf.square(true_xy - pred_xy) * object_mask * box_loss_scale * mix_w) / N
        wh_loss = tf.reduce_sum(tf.square(true_tw_th - pred_tw_th) * object_mask * box_loss_scale * mix_w) / N

        # shape: [N, 13, 13, 3, 1]
        conf_pos_mask = object_mask
        conf_neg_mask = (1 - object_mask) * ignore_mask
        conf_loss_pos = conf_pos_mask * tf.nn.sigmoid_cross_entropy_with_logits(labels=object_mask, logits=pred_conf_logits)
        conf_loss_neg = conf_neg_mask * tf.nn.sigmoid_cross_entropy_with_logits(labels=object_mask, logits=pred_conf_logits)
        # TODO: may need to balance the pos-neg by multiplying some weights
        conf_loss = conf_loss_pos + conf_loss_neg
        if self.use_focal_loss:
            alpha = 1.0
            gamma = 2.0
            # TODO: alpha should be a mask array if needed
            focal_mask = alpha * tf.pow(tf.abs(object_mask - tf.sigmoid(pred_conf_logits)), gamma)
            conf_loss *= focal_mask
        conf_loss = tf.reduce_sum(conf_loss * mix_w) / N

        # shape: [N, 13, 13, 3, 1]
        # whether to use label smooth
        if self.use_label_smooth:
            delta = 0.01
            label_target = (1 - delta) * y_true[..., 5:-1] + delta * 1. / self.class_num
        else:
            label_target = y_true[..., 5:-1]
        class_loss = object_mask * tf.nn.sigmoid_cross_entropy_with_logits(labels=label_target, logits=pred_prob_logits) * mix_w
        class_loss = tf.reduce_sum(class_loss) / N

        return xy_loss, wh_loss, conf_loss, class_loss

    
    def compute_loss(self, y_pred, y_true):
        '''
        param:
            y_pred: returned feature_map list by `forward` function: [feature_map_1, feature_map_2, feature_map_3]
            y_true: input y_true by the tf.data pipeline
        '''
        loss_xy, loss_wh, loss_conf, loss_class = 0., 0., 0., 0.
        anchor_group = [self.anchors[6:9], self.anchors[3:6], self.anchors[0:3]]

        # calc loss in 3 scales
        for i in range(len(y_pred)):
            result = self.loss_layer(y_pred[i], y_true[i], anchor_group[i])
            loss_xy += result[0]
            loss_wh += result[1]
            loss_conf += result[2]
            loss_class += result[3]
        total_loss = loss_xy + loss_wh + loss_conf + loss_class
        return [total_loss, loss_xy, loss_wh, loss_conf, loss_class]


    def broadcast_iou(self, true_box_xy, true_box_wh, pred_box_xy, pred_box_wh):
        '''
        maintain an efficient way to calculate the ios matrix between ground truth true boxes and the predicted boxes
        note: here we only care about the size match
        '''
        # shape:
        # true_box_??: [V, 2]
        # pred_box_??: [N, 13, 13, 3, 2]

        # shape: [N, 13, 13, 3, 1, 2]
        pred_box_xy = tf.expand_dims(pred_box_xy, -2)
        pred_box_wh = tf.expand_dims(pred_box_wh, -2)

        # shape: [1, V, 2]
        true_box_xy = tf.expand_dims(true_box_xy, 0)
        true_box_wh = tf.expand_dims(true_box_wh, 0)

        # [N, 13, 13, 3, 1, 2] & [1, V, 2] ==> [N, 13, 13, 3, V, 2]
        intersect_mins = tf.maximum(pred_box_xy - pred_box_wh / 2.,
                                    true_box_xy - true_box_wh / 2.)
        intersect_maxs = tf.minimum(pred_box_xy + pred_box_wh / 2.,
                                    true_box_xy + true_box_wh / 2.)
        intersect_wh = tf.maximum(intersect_maxs - intersect_mins, 0.)

        # shape: [N, 13, 13, 3, V]
        intersect_area = intersect_wh[..., 0] * intersect_wh[..., 1]
        # shape: [N, 13, 13, 3, 1]
        pred_box_area = pred_box_wh[..., 0] * pred_box_wh[..., 1]
        # shape: [1, V]
        true_box_area = true_box_wh[..., 0] * true_box_wh[..., 1]

        # [N, 13, 13, 3, V]
        iou = intersect_area / (pred_box_area + true_box_area - intersect_area + 1e-10)

        return iou


class TLClassifier_YOLOv3(object):
    def __init__(self):
        self.is_carla=rospy.get_param("is_carla")

        self.traffic_light_img_list = []
        self.traffic_light_scores = []

        #self.DEBUG_IMAGE = None


        self.input_size = [603, 603]
        self.model_path = './models/yolov3/yolov3.ckpt'
        self.anchor_path = './models/yolov3/yolo_anchors.txt'
        self.class_path = './models/yolov3/coco.names'
        self.sess = tf.Session()
        self.frame_count = 0

        self.anchors = parse_anchors(self.anchor_path)
        self.classes = read_class_names(self.class_path)
        num_class = len(self.classes)

        self.color_table = get_color_table(num_class)

        self.input_data = tf.placeholder(tf.float32, [1, self.input_size[1], self.input_size[0], 3], name='input_data')
        yolo_model = yolov3(num_class, self.anchors)
        with tf.variable_scope('yolov3'):
            pred_feature_maps = yolo_model.forward(self.input_data, False)
        pred_boxes, pred_confs, pred_probs = yolo_model.predict(pred_feature_maps)
        pred_scores = pred_confs * pred_probs

        self.boxes, self.scores, self.labels = gpu_nms(pred_boxes, pred_scores, num_class, max_boxes=30, score_thresh=0.4, nms_thresh=0.5)

        saver = tf.train.Saver()
        saver.restore(self.sess, self.model_path)
        print('Load done')
        
    def detect_traffic_lights(self, img_ori, confidence_level=0.2, detect_class_id=[9]):
        start_t = time.time()

        height_ori, width_ori = img_ori.shape[:2]
        img = cv2.resize(img_ori, tuple(self.input_size))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = np.asarray(img, np.float32)
        img = img[np.newaxis, :] / 255.

        boxes_, scores_, labels_ = self.sess.run([self.boxes, self.scores, self.labels], feed_dict={self.input_data: img})

        end_t = time.time()
        #print('Detection time:%.4fs'%(end_t - start_t))

        # rescale the coordinates to the original image
        boxes_[:, 0] *= (width_ori/float(self.input_size[0]))
        boxes_[:, 2] *= (width_ori/float(self.input_size[0]))
        boxes_[:, 1] *= (height_ori/float(self.input_size[1]))
        boxes_[:, 3] *= (height_ori/float(self.input_size[1]))

        self.traffic_light_img_list = []
        self.traffic_light_scores = []

        for i in range(len(boxes_)):
            x0, y0, x1, y1 = boxes_[i]

            x0 = max(0, int(x0))
            x1 = min(width_ori, int(x1))
            y0 = max(0, int(y0))
            y1 = min(height_ori, int(y1))

            if labels_[i] == 9 and scores_[i] >= confidence_level: 
                print(int(y0),int(y1), int(x0),int(x1))
                crop_img = img_ori[y0:y1, x0:x1].copy()
                #cv2.imshow("cropped", crop_img)
                #cv2.waitKey(0)
                self.traffic_light_img_list.append(crop_img)
                self.traffic_light_scores.append(scores_[i])

            #plot_one_box(img_ori, [x0, y0, x1, y1], label=self.classes[labels_[i]], score=scores_[i], color=self.color_table[labels_[i]])

        #cv2.imshow('Detection result', img_ori)
        #cv2.imwrite('out/%05d.png'%self.frame_count, img_ori)
        #self.DEBUG_IMAGE = img_ori

        self.frame_count+=1

        #print(labels_)
        #print(scores_)

        return len(self.traffic_light_scores)
    
    def get_classification(self):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        
        #return TrafficLight.UNKNOWN
        color_scores = [0.0, 0.0, 0.0, 0.0, 0.0]
        for i in range(min(3,len(self.traffic_light_scores))):
            result = self.red_green_yellow(self.traffic_light_img_list[i])
            rospy.logdebug("[tl_classifier] Top prob #%d: %.4f, %s", i, self.traffic_light_scores[i], result)
            color_scores[result] += self.traffic_light_scores[i]

        #rospy.logdebug("color_scores.len=%s, color_scores=%s", len(color_scores), color_scores)
        
        if color_scores[0] > 2 * color_scores[2]:
            return TrafficLight.RED
        elif color_scores[2] > 2 * color_scores[0]:
            return TrafficLight.GREEN
        
        return TrafficLight.UNKNOWN
        

    def findNoneZero(self, rgb_image):
        rows,cols,_ = rgb_image.shape
        counter = 0
        for row in range(rows):
            for col in range(cols):
                pixels = rgb_image[row,col]
                if sum(pixels)!=0:
                    counter = counter+1
        return counter

    
    def red_green_yellow(self, bgr_image, display=False):
        '''
        Determines the red , green and yellow content in each image using HSV and experimentally
        determined thresholds. Returns a Classification based on the values
        '''

        #image = cv2.imread(img_file)
        #image = cv2.resize(image,(32,32))

        #rgb_image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)

        #rgb_image = Image.open(img_file)  
        #rospy.loginfo("Detection Result:%f", rgb_image[1])
        
        #rgb_image.save('./test.jpg')
        bgr_image = cv2.resize(bgr_image,(32,32))
        rgb_image = cv2.cvtColor(bgr_image,cv2.COLOR_BGR2RGB)

        hsv = cv2.cvtColor(rgb_image,cv2.COLOR_RGB2HSV)

        sum_saturation = np.sum(hsv[:,:,1])# Sum the brightness values
        area = 32*32
        avg_saturation = sum_saturation / area #find average
        
        print(avg_saturation)
        sat_low_green = int(avg_saturation*0.5) 
        sat_low_red = int(avg_saturation*0.3) 
        sat_low_yellow = int(avg_saturation*0.7) 
        val_low = 180
        
        #print(sat_low, val_low)


        #Green
        lower_green = np.array([50,sat_low_green,val_low])
        upper_green = np.array([100,255,255])
        green_mask = cv2.inRange(hsv,lower_green,upper_green)
        green_result = cv2.bitwise_and(rgb_image,rgb_image,mask = green_mask)
        
        #Yellow
        lower_yellow = np.array([10,sat_low_yellow,val_low])
        upper_yellow = np.array([60,255,255])
        yellow_mask = cv2.inRange(hsv,lower_yellow,upper_yellow)
        yellow_result = cv2.bitwise_and(rgb_image,rgb_image,mask=yellow_mask)
        
        # Red 
        lower_red = np.array([150,sat_low_red,val_low])
        upper_red = np.array([180,255,255])
        red_mask1 = cv2.inRange(hsv,lower_red,upper_red)

        lower_red2 = np.array([0,sat_low_red,val_low])
        upper_red2 = np.array([30,255,255])
        red_mask2 = cv2.inRange(hsv,lower_red2,upper_red2)

        red_mask = np.bitwise_or(red_mask1, red_mask2)


        red_result = cv2.bitwise_and(rgb_image,rgb_image,mask = red_mask)
        '''
        if display==True:
            _,ax = plt.subplots(1,5,figsize=(20,10))
            ax[0].set_title('rgb image')
            ax[0].imshow(rgb_image)
            ax[1].set_title('red result')
            ax[1].imshow(red_result)
            ax[2].set_title('yellow result')
            ax[2].imshow(yellow_result)
            ax[3].set_title('green result')
            ax[3].imshow(green_result)
            ax[4].set_title('hsv image')
            ax[4].imshow(hsv)
            plt.show()
        '''
        sum_green = self.findNoneZero(green_result)
        sum_red = self.findNoneZero(red_result)
        sum_yellow = self.findNoneZero(yellow_result)
        #print(sum_red, sum_yellow, sum_green)
        
        if sum_red + sum_yellow >= 2 * sum_green:
            return 0
        elif sum_green >= sum_red:
            return 2
        else:
            return 4
        