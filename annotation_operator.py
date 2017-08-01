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

class AnnotationOperator:
    def __init__(self):
        self.ref_pt = []
        self.state = None
        self.__click_id = None
        self.__click_state = None
        self.__operating = False
        self.__mod_pt = [] # [stable, moving]
        self.__color_list = []
        self.guide_msgs = {'addanno':'Choose class and draw rectangles.',
                           'eraseanno':'Click a rectangle to erase.',
                           'modanno':'Click a vertex of a rectangle to reshape, inside point to move.'}

    def generate_colorlist(self, length):
        rand_list = [random.randint(0, 255) for i in xrange(length * 3)]
        for i in xrange(length):
            self.__color_list.append(rand_list[i * 3:(i + 1) * 3])

    def draw_rect(self, data):
        data.rected_image = data.image.copy()
        for name, rect in zip(data.class_vec, data.rect_vec):
            cv2.rectangle(data.rected_image, rect[0], rect[1], self.__color_list[int(data.id_reverse_dict[name])], 2)
            text_pos = (rect[0][0], rect[0][1] - 5)
            cv2.putText(data.rected_image, name, text_pos, cv2.FONT_HERSHEY_TRIPLEX, 1.0, self.__color_list[int(data.id_reverse_dict[name])])
        data.disp_image = data.rected_image.copy()

    ## return  clicked rect_id and (0:left-top | 1:left-bottom | 2:right-bottom | 3:right-top | 4: center |  5:none)
    def click_rect_check(self, pt, rects):
        vertex_thre = 8 # threthold for detecting vertex click in px
        found_center_click = []
        found_vertex_click = []
        for i, rect in enumerate(rects):
            if rect[0][0] < pt[0] < rect[1][0] and rect[0][1] < pt[1] < rect[1][1]:
                found_center_click.append(i)
            if abs(rect[0][0] - pt[0]) < vertex_thre:
                if abs(rect[0][1] - pt[1]) < vertex_thre:
                    found_vertex_click.append((i, 0, np.sqrt(np.square((rect[0][0] - pt[0])) + np.square((rect[0][1] - pt[1]))))) # (id, vertex, dist)
                elif abs(rect[1][1] - pt[1]) < vertex_thre:
                    found_vertex_click.append((i, 1, np.sqrt(np.square((rect[0][0] - pt[0])) + np.square((rect[1][1] - pt[1]))))) # (id, vertex, dist)
            elif abs(rect[1][0] - pt[0]) < vertex_thre:
                if abs(rect[1][1] - pt[1]) < vertex_thre:
                    found_vertex_click.append((i, 2, np.sqrt(np.square((rect[1][0] - pt[0])) + np.square((rect[1][1] - pt[1]))))) # (id, vertex, dist)
                elif abs(rect[0][1] - pt[1]) < vertex_thre:
                    found_vertex_click.append((i, 3, np.sqrt(np.square((rect[1][0] - pt[0])) + np.square((rect[0][1] - pt[1]))))) # (id, vertex, dist)

        if len(found_vertex_click) != 0:
            found_vertex_click.sort(key = lambda x:x[2])
            self.__click_id = found_vertex_click[0][0]
            self.__click_state = found_vertex_click[0][1]
        elif len(found_center_click) != 0:
            self.__click_id = found_center_click[0]
            self.__click_state = 4
        else:
            self.__click_id = 0
            self.__click_state = 5

    def click_annotate_rect(self, event, x, y, data, selected_val): # data should AnnotationContainer
        # event: 0: down, 1: move, 2:up, 3:wait
        EVENT_LBUTTONDOWN = 0
        EVENT_LBUTTONMOVE = 1
        EVENT_LBUTTONUP = 2
        EVENT_NOTHING = 3

        if event == EVENT_NOTHING:
            return

        ## Add annotation
        if self.state == "addanno":
            if event == EVENT_LBUTTONDOWN:
                self.ref_pt = [(x, y)]
                self.__operating = True
                return
            elif event == EVENT_LBUTTONUP:
                self.ref_pt.append((x, y))
                swap_pt = [(min(self.ref_pt[0][0], self.ref_pt[1][0]), min(self.ref_pt[0][1], self.ref_pt[1][1])),
                           (max(self.ref_pt[0][0], self.ref_pt[1][0]), max(self.ref_pt[0][1], self.ref_pt[1][1]))]
                data.class_vec.append(selected_val)
                data.rect_vec.append(swap_pt)
                self.draw_rect(data)

                self.__operating = False
                return
            elif self.__operating and event == EVENT_LBUTTONMOVE:
                tmp_pt = (x,y)
                swap_pt = [(min(self.ref_pt[0][0], tmp_pt[0]), min(self.ref_pt[0][1], tmp_pt[1])),
                           (max(self.ref_pt[0][0], tmp_pt[0]), max(self.ref_pt[0][1], tmp_pt[1]))]
                data.disp_image = data.rected_image.copy()
                cv2.rectangle(data.disp_image, swap_pt[0], swap_pt[1], self.__color_list[int(data.id_reverse_dict[selected_val])], 2)
                text_pos = (swap_pt[0][0], swap_pt[0][1] - 5)
                cv2.putText(data.disp_image, selected_val, text_pos, cv2.FONT_HERSHEY_TRIPLEX, 1.0, self.__color_list[int(data.id_reverse_dict[selected_val])])

        ## Erase annotation
        elif self.state == "eraseanno":
            if event == EVENT_LBUTTONDOWN and len(data.class_vec) != 0:
                self.click_rect_check((x, y), data.rect_vec)
                if self.__click_state != 5: #  selected
                    del data.class_vec[self.__click_id]
                    del data.rect_vec[self.__click_id]
                    self.draw_rect(data)
                    return

        ## Modify annotation
        elif self.state == "modanno":
            if event == EVENT_LBUTTONDOWN:
                self.click_rect_check((x, y), data.rect_vec)
                if self.__click_state == 4: # selected center
                    self.__mod_pt = [(x, y)]
                    self.__operating = True
                    return
                elif self.__click_state != 5: # selected vertex
                    stable_id = (self.__click_state + 2) % 4
                    stable_pt = []
                    if stable_id  in (0, 1): # left
                        stable_pt.append(data.rect_vec[self.__click_id][0][0])
                    else: # right
                        stable_pt.append(data.rect_vec[self.__click_id][1][0])
                    if stable_id  in (0, 3) : # top
                        stable_pt.append(data.rect_vec[self.__click_id][0][1])
                    else: # bottom
                        stable_pt.append(data.rect_vec[self.__click_id][1][1])
                    self.__mod_pt = [stable_pt, (0, 0)] # second is dammy
                    self.__operating = True
                    return
            elif event == EVENT_LBUTTONUP:
                new_rect = []
                if len(self.__mod_pt) == 1: # center
                    move_pt = (x - self.__mod_pt[0][0], y - self.__mod_pt[0][1])
                    new_rect = [(data.rect_vec[self.__click_id][0][0] + move_pt[0], data.rect_vec[self.__click_id][0][1] + move_pt[1]),
                                (data.rect_vec[self.__click_id][1][0] + move_pt[0], data.rect_vec[self.__click_id][1][1] + move_pt[1])]
                else: # vertex
                    self.__mod_pt.pop()
                    self.__mod_pt.append((x, y))
                    new_rect = [(min(self.__mod_pt[0][0], self.__mod_pt[1][0]), min(self.__mod_pt[0][1], self.__mod_pt[1][1])),
                                (max(self.__mod_pt[0][0], self.__mod_pt[1][0]), max(self.__mod_pt[0][1], self.__mod_pt[1][1]))]

                data.rect_vec[self.__click_id] = new_rect
                self.draw_rect(data)
                self.__operating = False
                return
            elif self.__operating and event == EVENT_LBUTTONMOVE:
                tmp_rect = []
                if len(self.__mod_pt) == 1: # center
                    move_pt = (x - self.__mod_pt[0][0], y - self.__mod_pt[0][1])
                    tmp_rect = [(data.rect_vec[self.__click_id][0][0] + move_pt[0], data.rect_vec[self.__click_id][0][1] + move_pt[1]),
                                (data.rect_vec[self.__click_id][1][0] + move_pt[0], data.rect_vec[self.__click_id][1][1] + move_pt[1])]
                else: # vertex
                    self.__mod_pt.pop()
                    self.__mod_pt.append((x, y))
                    tmp_rect = [(min(self.__mod_pt[0][0], self.__mod_pt[1][0]), min(self.__mod_pt[0][1], self.__mod_pt[1][1])),
                                (max(self.__mod_pt[0][0], self.__mod_pt[1][0]), max(self.__mod_pt[0][1], self.__mod_pt[1][1]))]

                tmp_rects = data.rect_vec[:]
                tmp_rects[self.__click_id] = tmp_rect
                data.disp_image = data.image.copy()
                for name, rect in zip(data.class_vec, tmp_rects):
                    cv2.rectangle(data.disp_image, rect[0], rect[1], self.__color_list[int(data.id_reverse_dict[name])], 2)
                    text_pos = (rect[0][0], rect[0][1] - 5)
                    cv2.putText(data.disp_image, name, text_pos, cv2.FONT_HERSHEY_TRIPLEX, 1.0, self.__color_list[int(data.id_reverse_dict[name])])
