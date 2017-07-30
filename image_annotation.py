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

image_topic_ = '/stereo/left/image_rect_color' ## topics of interest
save_directory = '/home/hiroto/data/conveni/train_data'
#save_directory = '/home/mizmizo/tmp_data/conveni_data/train_data2'
start_counter = 0 ## Start saving label from this number
keep_label = True ## true: reuse labels of previous image on next image

operator = None
container = None
window_name = 'image_annotation'
random.seed(5) ## good seed for 8 class

## <label_string 0.0 0 0.0 x_min y_min x_max y_max 0.0 0.0 0.0 0.0 0.0 0.0 0.0>

class AnnotationContainer:
    def __init__(self, save_directory, init_counter, keep_label = True):
        self.save_directory = '/home/hiroto/data/conveni/train_data'
        self.image = None ## image with dots
        self.rected_image = None ## image with dots and rects
        self.save_image = None ## original image
        self.id_dict = {}
        self.id_reverse_dict = {}
        self.class_vec = []
        self.rect_vec = []
        self.bridge = CvBridge()
        self.r_counter = 0
        self.w_counter = init_counter
        self.keep_label = keep_label

    def register_dict(self, class_path):
        if len(self.id_dict) != 0:
            printf("ID dictionary is not empty!")
            return
        for line in open(class_path, 'r'):
            line = line.rstrip("\n")
            pair = line.split()
            self.id_dict[pair[1]] = pair[0] ## key: id value: name
            self.id_reverse_dict[pair[0]] = pair[1] ## key: name value: id

    def erase_dict(self):
        self.id_dict = {}
        self.id_reverse_dict = {}

    def show_dict(self):
        print("Class list:")
        for x, name in sorted(self.id_dict.items()):
            print("ID:{0} Class:{1}".format(x, name))

    def show_label(self):
        print("Choosed labels:")
        for name, rect in zip(self.class_vec, self.rect_vec):
            print("{0} x_min:{1}, y_min:{2}, x_max:{3}, y_max:{4}".format(name, rect[0], rect[0][1], rect[1][0], rect[1][1]))

    def load_image_from_msg(self, msg, enc = "bgr8"):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, enc)
        except Exception as e:
            print (e)

        print("\nread  iteration: {0}".format(self.r_counter))
        print("write iteration: {0}".format(self.w_counter))
        self.r_counter += 1
        self.save_image = self.image.copy()
        stride = 16
        for j in xrange(0, self.image.shape[0], stride):
            for i in xrange(0, self.image.shape[1], stride):
                cv2.circle(self.image, (i, j), 2, (0, 255, 0), -1)
        self.rected_image = self.image.copy()

    def finish_imageproc(self):
        if not len(self.class_vec) == 0:
            ## write to file
            suffix_name = str(self.w_counter).zfill(6)
            text_file = open(self.save_directory + '/labels/' + suffix_name + '.txt', 'w')
            for name, rect in zip(self.class_vec, self.rect_vec):
                text_file.write('%s 0.0 0.0 0.0 %s %s %s %s 0.0 0.0 0.0 0.0 0.0 0.0 0.0\n' %
                                (name, float(rect[0][0]), float(rect[0][1]),
                                 float(rect[1][0]), float(rect[1][1])))
            text_file.close()
            cv2.imwrite(self.save_directory + '/images/' + suffix_name + '.jpg', self.save_image)
            self.w_counter += 1
            if not self.keep_label:
                self.class_vec = []
                self.rect_vec = []

class AnnotationOperator:
    def __init__(self):
        self.click_id = None
        self.click_state = None
        self.stable_pt = None
        self.operating = False
        self.ref_pt = []
        self.state = None
        self.color_list = []
        self.selected_id = None

    def usage(self):
        print("\nUsage:")
        print("1-1. Input class ID.")
        print("1-2. Click and drag to rectify object area.")
        print("1-3. Type 'o' for accept label, or type 'r' for retry selecting area")
        print("2. Type 's' for going to next image. selected labels will be saved unless it's empty.")
        print("\nSpecial commands(always available):")
        print("  l: show class list")
        print("  h: show this help")
        print("  c: show selected label")
        print("  q: exit this program")
        print("  a: enter add annotation mode")
        print("  e: enter erase annotation mode")
        print("  m: enter modify annotation mode")
        print("  s: go to next image")

    def generate_colorlist(self, length):
        rand_list = [random.randint(0, 255) for i in xrange(length * 3)]
        for i in xrange(length):
            self.color_list.append(rand_list[i * 3:(i + 1) * 3])

    def draw_rect(self, data):
        data.rected_image = data.image.copy()
        for name, rect in zip(data.class_vec, data.rect_vec):
            cv2.rectangle(data.rected_image, rect[0], rect[1], self.color_list[int(data.id_reverse_dict[name])], 2)

    def wait_command(self, container, preset_com = [""], ignore_state = "" , msg = ""):
        if msg != "":
            print(msg, end="")
            sys.stdout.flush()
        val = ''
        while True:
            key = cv2.waitKey(0) & 0xFF
            val = chr(key)
            if self.check_input(val, container) and self.state != ignore_state:
                return None
            elif val in preset_com:
                return val
            else:
                print("usable command is {0}".format(sorted(preset_com)))

    ## return  clicked rect_id and (0: center | 1:left-top | 2:left-bottom | 3:right-bottom | 4:right-top | 5:none)
    def click_rect_check(self, pt, rects):
        vertex_thre = 5 # threthold for detecting vertex click in px
        found_center_click = []
        found_vertex_click = []
        for i, rect in enumerate(rects):
            if rect[0][0] < pt[0] < rect[1][0] and rect[0][1] < pt[1] < rect[1][1]:
                found_center_click.append(i)
            if abs(rect[0][0] - pt[0]) < vertex_thre:
                if abs(rect[0][1] - pt[1]) < vertex_thre:
                    found_vertex_click.append((i, 1, np.sqrt(np.square((rect[0][0] - pt[0])) + np.square((rect[0][1] - pt[1]))))) # (id, vertex, dist)
                elif abs(rect[1][1] - pt[1]) < vertex_thre:
                    found_vertex_click.append((i, 2, np.sqrt(np.square((rect[0][0] - pt[0])) + np.square((rect[1][1] - pt[1]))))) # (id, vertex, dist)
            elif abs(rect[1][0] - pt[0]) < vertex_thre:
                if abs(rect[1][1] - pt[1]) < vertex_thre:
                    found_vertex_click.append((i, 3, np.sqrt(np.square((rect[1][0] - pt[0])) + np.square((rect[1][1] - pt[1]))))) # (id, vertex, dist)
                elif abs(rect[0][1] - pt[1]) < vertex_thre:
                    found_vertex_click.append((i, 4, np.sqrt(np.square((rect[1][0] - pt[0])) + np.square((rect[0][1] - pt[1]))))) # (id, vertex, dist)

        if len(found_vertex_click) != 0:
            found_vertex_click.sort(key = lambda x:x[2])
            self.click_id = found_vertex_click[0][0]
            self.click_state = found_vertex_click[0][1]
        elif len(found_center_click) != 0:
            self.click_id = found_center_click[0]
            self.click_state = 0
        else:
            self.click_id = 0
            self.click_state = 5

    # return True if state is changed
    def check_input(self, key_val, data):
        if key_val == 'l':
            data.show_dict()
            return False
        elif key_val == 'h':
            operator.usage()
            return False
        elif key_val == 'c':
            data.show_label()
            return False
        elif key_val == 'q':
            operator.state = "exit"
            return True
        elif key_val == 'a':
            operator.state = "waitaddanno"
            return True
        elif key_val == 'e':
            operator.state = "eraseanno"
            return True
        elif key_val == 'm':
            operator.state = "modanno"
            return True
        elif key_val == 's':
            operator.state = "norun"
            return True
        else:
            return False

    def click_annotate_rect(self, event, x, y, flags, param): # param should be AnnotationContainer, window_name
        data, window = param

        ## Add annotation
        if self.state == "addanno":
            if event == cv2.EVENT_LBUTTONDOWN:
                self.ref_pt = [(x, y)]
                self.operating = True
                return
            elif event == cv2.EVENT_LBUTTONUP:
                self.ref_pt.append((x, y))
                swap_pt = [(min(self.ref_pt[0][0], self.ref_pt[1][0]), min(self.ref_pt[0][1], self.ref_pt[1][1])),
                           (max(self.ref_pt[0][0], self.ref_pt[1][0]), max(self.ref_pt[0][1], self.ref_pt[1][1]))]
                self.ref_pt = swap_pt
                self.state = "waitaddanno"
                print("OK? if ok type 'o' for ok,  or 'r' for reject")
                sys.stdout.flush()
                self.operating = False
                return
            elif self.operating:
                tmp_pt = (x,y)
                swap_pt = [(min(self.ref_pt[0][0], tmp_pt[0]), min(self.ref_pt[0][1], tmp_pt[1])),
                           (max(self.ref_pt[0][0], tmp_pt[0]), max(self.ref_pt[0][1], tmp_pt[1]))]
                tmp_image = data.rected_image.copy()
                cv2.rectangle(tmp_image, swap_pt[0], swap_pt[1], self.color_list[int(self.selected_id)], 2)
                cv2.imshow(window, tmp_image)
                cv2.waitKey(1)

        ## Erase annotation
        elif self.state == "eraseanno":
            if event == cv2.EVENT_LBUTTONDOWN and len(data.class_vec) != 0:
                erase_pt = (x, y)
                self.click_rect_check(erase_pt, data.rect_vec)
                if self.click_state != 5: #  selected
                    del data.class_vec[self.click_id]
                    del data.rect_vec[self.click_id]
                    self.draw_rect(data)
                    cv2.imshow(window, data.rected_image)
                    return

        ## Modify annotation
        elif self.state == "modanno":
            if event == cv2.EVENT_LBUTTONDOWN:
                mod_pt = [(x, y)]
                self.click_rect_check(mod_pt)
                #todo
                if self.click_state != 5: # selected
                    self.operating = True
                    return
            elif event == cv2.EVENT_LBUTTONUP:
                self.ref_pt.append((x, y))
                swap_pt = [(min(ref_pt[0][0], ref_pt[1][0]), min(ref_pt[0][1], ref_pt[1][1])),
                           (max(ref_pt[0][0], ref_pt[1][0]), max(ref_pt[0][1], ref_pt[1][1]))]
                ref_pt = swap_pt

                cv2.rectangle(image, ref_pt[0], ref_pt[1], self.color_list[int(self.selected_id)], 2)
                cv2.imshow(window_name, image)
                self.state = "modanno"
                print("OK? if ok type 'a', 'm', or 'e'")
                sys.stdout.flush()
                self.operating = False
                return
            elif self.operating:
                tmp_rect = rect_vec[self.click_id]
                tmp_image = image.copy()
                cv2.rectangle(tmp_image, swap_pt[0], swap_pt[1], self.color_list[int(self.selected_id)], 2)
                cv2.imshow(window_name, tmp_image)
                cv2.waitKey(1)


def read_rosbag(bag_path, class_path):
    global container, operator

    container = AnnotationContainer(save_directory, start_counter, keep_label)
    operator = AnnotationOperator()
    bag = rosbag.Bag(bag_path, 'r')

    ## setup container
    container.register_dict(class_path)
    container.show_dict()
    operator.generate_colorlist(len(container.id_dict))

    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, operator.click_annotate_rect, [container, window_name])

    ## read each image in rosbag
    for topic, msg, t in bag.read_messages(topics=[image_topic_]):

        ### exit program
        if operator.state == "exit":
            bag.close()
            return

        ### preparation
        container.load_image_from_msg(msg)
        operator.draw_rect(container)
        cv2.imshow(window_name, container.rected_image)

        ### choose operation
        operator.wait_command(container, msg = "Choose operation for this image : ")
        print(operator.state)

        ### skip image or exit
        if operator.state == "norun":
            container.finish_imageproc()
            continue
        elif operator.state == "exit":
            continue

        ### operate on image
        while not operator.state in ("norun", "exit"):
            ### Add annotation
            if operator.state == "waitaddanno":
                com = operator.wait_command(container,
                                            preset_com = container.id_dict.keys(),
                                            ignore_state = "waitaddanno",
                                            msg = "Enter id num : ")
                if com: ## not moved to other state
                    print(container.id_dict[com])
                    operator.selected_id = com
                    print("draw rect")
                    sys.stdout.flush()
                    while True:
                        operator.state = "addanno"
                        com = operator.wait_command(container,
                                                    preset_com = ["o", "r"])
                        if not com:
                            print("You can't change mode now! Answer 'o' or 'r' first")
                            operator.state = "waitaddanno"
                            continue
                        elif com == 'o' and operator.state == "waitaddanno":
                            print("add label")
                            sys.stdout.flush()
                            container.class_vec.append(container.id_dict[operator.selected_id])
                            container.rect_vec.append(operator.ref_pt)
                            operator.draw_rect(container)
                            cv2.imshow(window_name, container.rected_image)
                            operator.state = "waitaddanno"
                            break
                        elif com == 'r' and operator.state == "waitaddanno":
                            print("Retry")
                            sys.stdout.flush()
                            cv2.imshow(window_name, container.rected_image)
                            operator.state = "waitaddanno"
                            break

            ### Erase or modify annotation
            if operator.state in ("eraseanno", "modanno"):
                operator.wait_command(container,
                                      ignore_state = operator.state,
                                      msg = "Click rectangle.\n")

            ## go-to-next image
            if operator.state == "norun":
                container.finish_imageproc()

    bag.close()

def main(argv):
    if len(argv) < 3:
        print('Usage: image_annotation.py <rosbag> <classlist>')
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
