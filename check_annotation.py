#!/usr/bin/env python

from __future__ import print_function

import numpy as np
import cv2
import sys
import os
import random

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

id_dict = {}
window_name = 'annotation_check'
color_list = []

## == params == ##
random.seed(5)
show_time = 0 # show each image and label for show_time ms
save_image = False
save_dir = os.environ["HOME"] + "/tmp_data/conveni_data/for_drive/original_data/fcsc/with_annotation/"
save_step = 1
start = 0

## <label_string 0.0 0 0.0 x_min y_min x_max y_max 0.0 0.0 0.0 0.0 0.0 0.0 0.0>

def show_dict(dict):
    print("Class list:")
    for x, name in sorted(dict.items()):
        print("ID:{0} Class:{1}".format(x, name))

def check_data(data_directory, class_path):
    global id_dict

    ## register class list
    for line in open(class_path, 'r'):
        line = line.rstrip("\n")
        pair = line.split()
        id_dict[pair[0]] = pair[1]
    show_dict(id_dict)

    ## generate color list
    rand_list = [random.randint(0, 255) for i in xrange(len(id_dict) * 3)]
    for i in xrange(len(id_dict)):
        color_list.append(rand_list[i * 3:(i + 1) * 3])

    cv2.namedWindow(window_name, cv2.WINDOW_KEEPRATIO | cv2.WINDOW_NORMAL)

    ## get image and label name list
    image_list = sorted(os.listdir(data_directory + "/images/"))
    label_list = sorted(os.listdir(data_directory + "/labels/"))
    load_cnt = 0

    print("Start checking annotation, type 'q' for quiting, any other key for next image.")

    ## load label and visualize
    for image_name, label_name in zip(image_list[start:], label_list[start:]):
        image = cv2.imread(data_directory + "/images/" + image_name)
        for line in open(data_directory + "/labels/" + label_name, "r"):
            line = line.rstrip("\n")
            label = line.split()[0]
            rect_str = line.split()[4:8]
            rect = []
            for i in xrange(len(rect_str)):
                rect.append(int(float(rect_str[i])))
            roi = [(rect[0], rect[1]), (rect[2], rect[3])]

            cv2.rectangle(image, roi[0], roi[1], color_list[int(id_dict[label])], 2)
            roi.append((roi[0][0], roi[0][1] - 5)) # offset for text
            cv2.putText(image, label, roi[2], cv2.FONT_HERSHEY_TRIPLEX, 1.0, color_list[int(id_dict[label])])

        sys.stdout.write("\rShowing %s, %s" % (image_name, label_name))
        sys.stdout.flush()
        cv2.imshow(window_name, image)
        if save_image and (load_cnt % save_step) == 0:
            save_name = save_dir + image_name
            cv2.imwrite(save_name, image)
        load_cnt += 1
        key = cv2.waitKey(show_time) & 0xFF
        if chr(key) == 'q':
            return

def main(argv):
    if len(argv) < 3:
        print('provide path to data and class list')
        return

    print('\033[33mchecking directory\033[0m')
    try:
        isdir = os.path.isdir(argv[1])
        if not isdir:
            raise ValueError('\033[31mdata directory not found\033[0m')
    except ValueError, e:
        print(e)
        raise SystemExit

    check_data(argv[1], argv[2])

if __name__ == "__main__":
    main(sys.argv)
