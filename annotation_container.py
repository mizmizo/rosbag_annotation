#!/usr/bin/env python

from __future__ import print_function
import roslib
import rospy
import rosbag

import numpy as np
import cv2
import sys
import os
import random

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

random.seed(5)

## <label_string 0.0 0 0.0 x_min y_min x_max y_max 0.0 0.0 0.0 0.0 0.0 0.0 0.0>

class AnnotationContainer:
    def __init__(self, save_directory, init_counter, keep_label = True):
        self.__save_directory = '/home/hiroto/data/conveni/train_data'
        self.image = None ## image with dots
        self.rected_image = None ## image with dots and rects
        self.disp_image = None ## image to display
        self.__save_image = None ## original image
        self.id_dict = {}
        self.id_reverse_dict = {}
        self.class_vec = []
        self.rect_vec = []
        self.__bridge = CvBridge()
        self.r_counter = 0
        self.w_counter = init_counter
        self.keep_label = keep_label

    def register_dict(self, class_path):
        if len(self.id_dict) != 0 or len(self.id_reverse_dict) != 0:
            printf("ID dictionary is not empty!")
            return
        for line in open(class_path, 'r'):
            line = line.rstrip("\n")
            pair = line.split()
            self.id_dict[pair[1]] = pair[0] ## key: id value: name
            self.id_reverse_dict[pair[0]] = pair[1] ## key: name value: id

    def load_image_from_msg(self, msg, enc = "bgr8"):
        try:
            self.image = self.__bridge.imgmsg_to_cv2(msg, enc)
        except Exception as e:
            print (e)

        print("\nread  iteration: {0}".format(self.r_counter))
        print("write iteration: {0}".format(self.w_counter))
        self.r_counter += 1
        self.__save_image = self.image.copy()
        stride = 16
        for j in xrange(0, self.image.shape[0], stride):
            for i in xrange(0, self.image.shape[1], stride):
                cv2.circle(self.image, (i, j), 2, (100, 100, 100), -1)
        self.rected_image = self.image.copy()
        #self.disp_image = self.image.copy()

    def finish_imageproc(self, save = True):
        if not len(self.class_vec) == 0 and save:
            ## write to file
            suffix_name = str(self.w_counter).zfill(6)
            text_file = open(self.__save_directory + '/labels/' + suffix_name + '.txt', 'w')
            for name, rect in zip(self.class_vec, self.rect_vec):
                text_file.write('%s 0.0 0.0 0.0 %s %s %s %s 0.0 0.0 0.0 0.0 0.0 0.0 0.0\n' %
                                (name, float(rect[0][0]), float(rect[0][1]),
                                 float(rect[1][0]), float(rect[1][1])))
            text_file.close()
            cv2.imwrite(self.__save_directory + '/images/' + suffix_name + '.jpg', self.__save_image)
            self.w_counter += 1
        if not self.keep_label:
            self.class_vec = []
            self.rect_vec = []
