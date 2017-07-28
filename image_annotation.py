#!/usr/bin/env python

from __future__ import print_function
import roslib
import rospy
import rosbag

import numpy as np
import cv2
import sys
import os
import argparse
import random

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

image_topic_ = '/stereo/left/image_rect_color' ## topics of interest
save_directory = '/home/mizmizo/tmp_data/conveni_data/train_data2'
w_counter = 0 ## Start saving label from this number

ref_pt = []
image = None
rected_image = None
save_image = None
selected_id = None
state = None ## norun, addanno, modanno
id_dict = {}
id_reverse_dict = {}
window_name = 'image_annotation'
random.seed(5) ## good seed for 8 class
color_list = []
operating = False
class_vec = []
rect_vec = []


## <label_string 0.0 0 0.0 x_min y_min x_max y_max 0.0 0.0 0.0 0.0 0.0 0.0 0.0>

def usage():
    print("\n\n1. Input class ID. Type 'l' for check class list.")
    print("2. Click and drag to rectify object area.")
    print("3. Type 'a' for accept label, or type 'e' for retry selecting area")
    print("4. Type 'a' for adding a label, 'n' for going to next image, or 'q' for quiting annotation")
    print("5. Input ID for adding next label, or type n for going to next image")
    print("\nOther commands(only enable in ID select phase):")
    print("  r: reset and erase the label file for current image")
    print("  h: show this help")
    print("  s: skip the image\n")

def show_dict(dict):
    print("Class list:")
    for x, name in sorted(dict.items()):
        print("ID:{0} Class:{1}".format(x, name))

def show_label():
    global image, class_vec, rect_vec
    print("Choosed labels:")
    for name, rect in zip(class_vec, rect_vec):
        print("{0} x_min:{1}, y_min:{2}, x_max:{3}, y_max:{4}".format(name, rect[0], rect[0][1], rect[1][0], rect[1][1]))

def draw_rect():
    global rected_image, image
    rected_image = image.copy()
    for name, rect in zip(class_vec, rect_vec):
        cv2.rectangle(rected_image, rect[0], rect[1], color_list[int(id_reverse_dict[name])], 2)

# return True if state is changed
def check_input(key_val):
    global state
    if key_val == 'l':
        show_dict(id_dict)
        return False
    elif key_val == 'h':
        usage()
        return False
    elif key_val == 'c':
        show_label()
        return False
    elif key_val == 'q':
        state = "exit"
        return True
    elif key_val == 'a':
        state = "waitaddanno"
        return True
    elif key_val == 's':
        state = "norun"
        return True
    elif key_val == 'm':
        state = "waitmodanno"
        return True
    else:
        return False

def finish_imageproc():
    global class_vec, rect_vec, w_counter
    if not len(class_vec) == 0:
        ## write to file
        suffix_name = str(w_counter).zfill(6)
        text_file = open(save_directory + '/labels/' + suffix_name + '.txt', 'w')
        for name, rect in zip(class_vec, rect_vec):
            text_file.write('%s 0.0 0.0 0.0 %s %s %s %s 0.0 0.0 0.0 0.0 0.0 0.0 0.0\n' %
                            (name, float(rect[0][0]), float(rect[0][1]),
                             float(rect[1][0]), float(rect[1][1])))
        text_file.close()
        cv2.imwrite(save_directory + '/images/' + suffix_name + '.jpg', save_image)
        class_vec = []
        rect_vec = []
        w_counter += 1


def click_annotate_rect(event, x, y, flags, param):
    global state, ref_pt, operating, rected_image

    ## Add annotation
    if state == "addanno":
        if event == cv2.EVENT_LBUTTONDOWN:
            ref_pt = [(x, y)]
            operating = True
            return
        elif event == cv2.EVENT_LBUTTONUP:
            ref_pt.append((x, y))
            swap_pt = [(min(ref_pt[0][0], ref_pt[1][0]), min(ref_pt[0][1], ref_pt[1][1])),
                       (max(ref_pt[0][0], ref_pt[1][0]), max(ref_pt[0][1], ref_pt[1][1]))]
            ref_pt = swap_pt
            state = "waitaddanno"
            print("OK? if ok type 'o' or 'r'")
            sys.stdout.flush()
            operating = False
            return
        elif operating:
            tmp_pt = (x,y)
            swap_pt = [(min(ref_pt[0][0], tmp_pt[0]), min(ref_pt[0][1], tmp_pt[1])),
                       (max(ref_pt[0][0], tmp_pt[0]), max(ref_pt[0][1], tmp_pt[1]))]
            tmp_image = rected_image.copy()
            cv2.rectangle(tmp_image, swap_pt[0], swap_pt[1], color_list[int(selected_id)], 2)
            cv2.imshow(window_name, tmp_image)
            cv2.waitKey(1)
    ## Modify annotation
    elif state == "modanno":
        if event == cv2.EVENT_LBUTTONDOWN:
            ref_pt = [(x, y)]
            operating = True
            return
        elif event == cv2.EVENT_LBUTTONUP:
            ref_pt.append((x, y))
            swap_pt = [(min(ref_pt[0][0], ref_pt[1][0]), min(ref_pt[0][1], ref_pt[1][1])),
                       (max(ref_pt[0][0], ref_pt[1][0]), max(ref_pt[0][1], ref_pt[1][1]))]
            ref_pt = swap_pt

            cv2.rectangle(image, ref_pt[0], ref_pt[1], color_list[int(selected_id)], 2)
            cv2.imshow(window_name, image)
            state = "waitmodanno"
            print("OK? if ok type 'a', 'm', or 'e'")
            sys.stdout.flush()
            operating = False
            return
        elif operating:
            tmp_pt = (x,y)
            swap_pt = [(min(ref_pt[0][0], tmp_pt[0]), min(ref_pt[0][1], tmp_pt[1])),
                       (max(ref_pt[0][0], tmp_pt[0]), max(ref_pt[0][1], tmp_pt[1]))]
            tmp_image = image.copy()
            cv2.rectangle(tmp_image, swap_pt[0], swap_pt[1], color_list[int(selected_id)], 2)
            cv2.imshow(window_name, tmp_image)
            cv2.waitKey(1)


def read_rosbag(bag_path, class_path):
    global id_dict, w_counter
    bag = rosbag.Bag(bag_path, 'r')
    bridge = CvBridge()

    ## register class list
    for line in open(class_path, 'r'):
        line = line.rstrip("\n")
        pair = line.split()
        id_dict[pair[1]] = pair[0] ## key: id value: name
        id_reverse_dict[pair[0]] = pair[1] ## key: name value: id
    show_dict(id_dict)

    ## generate color list
    rand_list = [random.randint(0, 255) for i in xrange(len(id_dict) * 3)]
    for i in xrange(len(id_dict)):
        color_list.append(rand_list[i * 3:(i + 1) * 3])

    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, click_annotate_rect)

    r_counter = 0

    ## read each image in rosbag
    for topic, msg, t in bag.read_messages(topics=[image_topic_]):
        global image, state, class_vec, rect_vec, rected_image, save_image

        if state == "exit":
            bag.close()
            return

        ### preparation
        try:
            image = bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print (e)

        print("\nread  iteration: {0}".format(r_counter))
        print("write iteration: {0}".format(w_counter))
        r_counter += 1
        save_image = image.copy()
        stride = 16
        for j in xrange(0, image.shape[0], stride):
            for i in xrange(0, image.shape[1], stride):
                cv2.circle(image, (i, j), 2, (0, 255, 0), -1)
        rected_image = image.copy()
        cv2.imshow(window_name, image)
        saved = False

        ### choose operation
        print("Choose operation for this image : ", end="")
        sys.stdout.flush()
        val = ''
        while True:
            key = cv2.waitKey(0) & 0xFF
            val = chr(key)
            if check_input(val):
                print(state)
                break

        if state == "norun":
            finish_imageproc()
            continue
        elif state == "exit":
            continue

        ### operate on image
        while True:
            global selected_id

            if state == "waitaddanno":
                print("Enter id num : ", end="")
                sys.stdout.flush()
                val = ''
                # get label id
                while True:
                    key = cv2.waitKey(0) & 0xFF
                    val = chr(key)
                    if check_input(val):
                        break
                    elif val in id_dict.keys():
                        print(id_dict[val])
                        selected_id = val
                        break

                if state == "waitaddanno":
                    print("draw rect")
                    sys.stdout.flush()
                    state = "addanno"
                    val = ''
                    while True:
                        key = cv2.waitKey(0) & 0xFF
                        val = chr(key)
                        if check_input(val) and state != "waitaddanno":
                            break
                        if val == 'o':
                            print("add label")
                            sys.stdout.flush()
                            class_vec.append(id_dict[selected_id])
                            rect_vec.append(ref_pt)
                            draw_rect()
                            cv2.imshow(window_name, rected_image)
                            break
                        elif val == 'r':
                            print("Retry")
                            sys.stdout.flush()
                            cv2.imshow(window_name, rected_image)
                            state = "waitaddanno"
                            break
                        else:
                            print("answer in 'o' or 'r'")

                if state != "waitaddanno":
                    break

        if state == "norun":
            finish_imageproc()
            continue
        elif state == "exit":
            continue

    bag.close()


def main(argv):
    if len(argv) < 3:
        print('provide path to rosbag and class list')
        return

    print('\033[33mchecking directory\033[0m')
    try:
        isdir = os.path.isdir(save_directory)
        if not isdir:
            raise ValueError('\033[31msaving directory not found\033[0m')
    except ValueError, e:
        print(e)
        raise SystemExit

    print("bag file:{0} class list:{1}".format(argv[1], argv[2]))
    read_rosbag(argv[1], argv[2])

if __name__ == "__main__":
    main(sys.argv)
