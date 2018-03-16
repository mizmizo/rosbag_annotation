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

## === Annotated Data container === ##

## <label_string 0.0 0 0.0 x_min y_min x_max y_max 0.0 0.0 0.0 0.0 0.0 0.0 0.0>

class AnnotationContainer:
    def __init__(self, save_directory, init_counter, keep_label = True):
        self.dot_image = None ## image with dots
        self.rected_image = None ## image with dots and rects
        self.disp_image = None ## image to display
        self.save_image = None ## original image
        self.id_dict = {}
        self.id_reverse_dict = {}
        self.class_vec = []
        self.rect_vec = []
        self.__bridge = CvBridge()
        self.r_counter = 0
        self.w_counter = init_counter
        # check directory
        self.__save_directory = save_directory
        for dname in ("/images/", "/labels/"):
            if not os.path.isdir(self.__save_directory + dname):
                os.mkdir(self.__save_directory + dname)

    def register_dict(self, class_path):
        if len(self.id_dict) != 0 or len(self.id_reverse_dict) != 0:
            printf("ID dictionary is not empty!")
            return
        for line in open(class_path, 'r'):
            line = line.rstrip("\n")
            pair = line.split()
            self.id_dict[pair[1]] = pair[0] ## key: id value: name
            self.id_reverse_dict[pair[0]] = pair[1] ## key: name value: id

    def show_dict(self):
        print("Class list:")
        for x, name in sorted(self.id_dict.items()):
            print("ID:{0} Class:{1}".format(x, name))

    def load_image_from_msg(self, msg, enc = "bgr8"):
        try:
            self.dot_image = self.__bridge.imgmsg_to_cv2(msg, enc)
        except Exception as e:
            print (e)

        self.r_counter += 1
        self.save_image = self.dot_image.copy()
        stride = 16
        for j in xrange(0, self.dot_image.shape[0], stride):
            for i in xrange(0, self.dot_image.shape[1], stride):
                cv2.circle(self.dot_image, (i, j), 2, (100, 100, 100), -1)

    def load_image_from_path(self, image_path, label_path):
        try:
            self.dot_image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        except Exception as e:
            print (e)

        try:
            for line in open(label_path, 'r'):
                line = line.rstrip("\n")
                label = line.split()[0]
                rect_str = line.split()[4:8]
                rect = []
                for i in xrange(len(rect_str)):
                    rect.append(int(float(rect_str[i])))
                self.rect_vec.append([(rect[0], rect[1]), (rect[2], rect[3])])
                self.class_vec.append(label)
        except Exception as e:
            print (e)

        self.r_counter += 1
        self.save_image = self.dot_image.copy()
        stride = 16
        for j in xrange(0, self.dot_image.shape[0], stride):
            for i in xrange(0, self.dot_image.shape[1], stride):
                cv2.circle(self.dot_image, (i, j), 2, (100, 100, 100), -1)

    def finish_imageproc(self, save = True, keep_label = True):
        if not len(self.class_vec) == 0 and save:
            ## write to file
            suffix_name = str(self.w_counter).zfill(6)
            text_file = open(self.__save_directory + '/labels/' + suffix_name + '.txt', 'w')
            for name, rect in zip(self.class_vec, self.rect_vec):
                text_file.write('%s 0.0 0 0.0 %s %s %s %s 0.0 0.0 0.0 0.0 0.0 0.0 0.0\n' %
                                (name, float(rect[0][0]), float(rect[0][1]),
                                 float(rect[1][0]), float(rect[1][1])))
            text_file.close()
            cv2.imwrite(self.__save_directory + '/images/' + suffix_name + '.jpg', self.save_image)
            self.w_counter += 1
        if not keep_label:
            self.class_vec = []
            self.rect_vec = []
