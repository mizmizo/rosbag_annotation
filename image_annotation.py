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

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

## topics of interest
image_topic_ = '/stereo/left/image_rect_color'
save_directory = '/home/mizmizo/tmp_data/conveni_data/train_data'
window_name = 'image_annotation'
w_counter = 0

ref_pt = []
image = None
selected_id = None
run_cb = False
id_dict = {}

## <label_string 0.0 0 0.0 x_min y_min x_max y_max 0.0 0.0 0.0 0.0 0.0 0.0 0.0>

def usage():
    print("\n\n1. Input class ID. Type 'l' for check class list.")
    print("2. Click and drag to rectify object area.")
    print("3. Type 'a' for adding label, or type 'e' for retry selecting area")
    print("4. Input ID for adding next label, or type n for going to next image")
    print("\nOther commands(only enable in ID select phase):")
    print("  r: reset and erase the label file for current image")
    print("  h: show this help")
    print("  s: skip the image\n")

def show_dict(dict):
    print("Class list:")
    for x, name in sorted(dict.items()):
        print("ID:{0} Class:{1}".format(x, name))

def check_input(key_val):
    if key_val == 'l':
        show_dict(id_dict)
        return False
    elif key_val == 'r':
        # TODO remove label
        return False
    elif key_val == 'h':
        usage()
        return False
    else:
        return True

def click_annotate_rect(event, x, y, flags, param):
    global run_cb, ref_pt

    if not run_cb:
        return

    if event == cv2.EVENT_LBUTTONDOWN:
        ref_pt = [(x, y)]
    elif event == cv2.EVENT_LBUTTONUP:
        ref_pt.append((x, y))

        swap_pt = [(min(ref_pt[0][0], ref_pt[1][0]), min(ref_pt[0][1], ref_pt[1][1])),
                   (max(ref_pt[0][0], ref_pt[1][0]), max(ref_pt[0][1], ref_pt[1][1]))]
        ref_pt = swap_pt

        color = [0, 0, 0]
        color[int(selected_id) % 3] = 255
        cv2.rectangle(image, ref_pt[0], ref_pt[1], color, 2)
        cv2.imshow(window_name, image)
        run_cb = False
        print("OK? a or e")
        sys.stdout.flush()

def read_rosbag(bag_path, class_path):
    global id_dict, w_counter
    bag = rosbag.Bag(bag_path, 'r')
    bridge = CvBridge()

    ## register class list
    for line in open(class_path, 'r'):
        line = line.rstrip("\n")
        pair = line.split()
        id_dict[pair[1]] = pair[0]
    show_dict(id_dict)

    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, click_annotate_rect)

    r_counter = 0

    for topic, msg, t in bag.read_messages(topics=[image_topic_]):
        global image, save_image
        try:
            image = bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print (e)

        print("read  iteration: {0}".format(r_counter))
        print("write iteration: {0}".format(w_counter))
        save_image = image.copy()
        suffix_name = str(w_counter).zfill(6)

        stride = 16
        for j in xrange(0, image.shape[0], stride):
            for i in xrange(0, image.shape[1], stride):
                cv2.circle(image, (i, j), 2, (0, 255, 0), -1)
        prev_image = image.copy()
        cv2.imshow(window_name, image)

        ## labelling

        saved = False
        while True:
            global selected_id, run_cb

            print("Enter id num : ", end="")
            sys.stdout.flush()
            key = cv2.waitKey(0) & 0xFF
            val = chr(key)
            flag = check_input(val)
            if flag:
                if val in id_dict.keys():
                    print(id_dict[val])
                    selected_id = val
                elif val == 's':
                    break
                else:
                    flag = False

            if flag:
                print("draw rect")
                sys.stdout.flush()
                run_cb = True
                val = ''
                while val != 'a' and val != 'e':
                    key = cv2.waitKey(0) & 0xFF
                    val = chr(key)
                    if val == 'a':
                        print("Write to file")
                        sys.stdout.flush()

                        ## write to file
                        text_file = open(save_directory + '/labels/' + suffix_name + '.txt', 'a')
                        text_file.write('%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s\n' %
                                        (id_dict[selected_id], 0.0, 0, 0.0, float(ref_pt[0][0]), float(ref_pt[0][1]),
                                         float(ref_pt[1][0]), float(ref_pt[1][1]), 0.0, 0.0, \
                                         0.0, 0.0, 0.0, 0.0, 0.0))
                        text_file.close()
                        cv2.imwrite(save_directory + '/images/' + suffix_name + '.jpg', save_image)
                        saved = True
                        prev_image = image.copy()
                        cv2.imshow(window_name, image)
                    elif val == 'e':
                        print("Retry")
                        sys.stdout.flush()
                        image = prev_image.copy()
                        cv2.imshow(window_name, image)
                    else:
                        print("answer in 'a' or 'e'")
            print("'n': next image\n'a': next object in same image\n'q': finish annotation")
            sys.stdout.flush()
            val = ''
            while val != 'n' and val != 'a' and val !='q':
                key = cv2.waitKey(0) & 0xFF
                val = chr(key)
            if val == 'n':
                break
            elif val == 'q':
                bag.close()
                return
        if saved:
            w_counter += 1
        r_counter += 1

    bag.close()


def main(argv):
    if len(argv) < 3:
        print('provide path to rosbag')
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
