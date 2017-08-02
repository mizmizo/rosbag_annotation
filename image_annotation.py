#!/usr/bin/python

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

import annotation_container as ac
import annotation_operator as ao

from kivy.app import App
from kivy.uix.widget import Widget
from kivy.properties import StringProperty, ListProperty, BooleanProperty, NumericProperty, ObjectProperty
from kivy.uix.spinner import Spinner
from kivy.uix.label import Label
from kivy.uix.image import Image
from kivy.clock import Clock
from kivy.graphics.texture import Texture
from kivy.animation import Animation
from kivy.uix.screenmanager import Screen

random.seed(5)

container = None
operator = None

class AnnotationWidget(Screen, Label, Image):
    state = StringProperty()
    label_list = ListProperty()
    selected_label = StringProperty()
    w_counter  = NumericProperty(0) ## just for display
    r_counter  = NumericProperty(0) ## just for display
    guide_msg = StringProperty()
    image_view = ObjectProperty(None)
    class_label_spinner = ObjectProperty(None)

    def __init__(self, **kwargs):
        super(AnnotationWidget, self).__init__(**kwargs)
        Clock.schedule_interval(self.update, 1.0 / 60)
        self.state = 'addanno'
        self.label_list = []
        self.guide_msg = ""
        self.bag_path = ''
        self.class_path = ''
        self.image_topic = ''
        self.save_directory = ''
        self.bag = None
        self.container = container
        self.operator = operator
        self.image_msgs = None
        self.touch_pos = []
        self.touch_event = None # 0: down, 1: move, 2:up, 3:wait

    def update(self, dt):
        if self.touch_pos != self.ids.image_view.touch_pos or self.touch_event != self.ids.image_view.touch_event:
            self.touch_pos = self.ids.image_view.touch_pos
            self.touch_event = self.ids.image_view.touch_event
            self.operator.click_annotate_rect(self.touch_event,
                                              self.touch_pos[0], self.touch_pos[1],
                                              self.container, self.selected_label)
            self.ids.image_view.setImage(self.container.disp_image)

    def resetupdate(self):
        self.touch_event = 3

    def startAnnotation(self, sm, bag, topic, class_list, save_dir, start_w, start_r):
        # bagfile
        try:
            self.bag = rosbag.Bag(bag.text, 'r')
        except Exception as e:
            self.guide_msg = "Bag File Error!"
            return
        # image topic
        if topic.text == "":
            self.guide_msg = "Topic name is Empty!"
            return
        else:
            self.image_topic = topic.text
        # class list
        if class_list.text == "":
            self.guide_msg = "Class list is Empty!"
            return
        else:
            self.class_path = class_list.text

        # save directory
        if save_dir.text == "":
            self.guide_msg = "Save directory is Empty!"
            return
        elif save_dir.text.find("/") != len(save_dir.text) - 1:
            self.save_directory = save_dir.text + "/"
        else:
            self.save_directory = save_dir.text
        if not os.path.isdir(self.save_directory):
            self.guide_msg = "Invalid save directory name!"
            return
        # start writing counter
        try:
            self.w_counter = int(start_w.text)
        except Exception as e:
            self.guide_msg = "Enter integer in start counter!"
            return

        # start writing counter
        try:
            self.r_counter = int(start_r.text)
        except Exception as e:
            self.guide_msg = "Enter integer in start counter!"
            return

        # preparation
        self.container = ac.AnnotationContainer(self.save_directory, self.w_counter, True)
        self.container.register_dict(self.class_path)
        for x, name in sorted(self.container.id_dict.items()):
            self.label_list.append(name)
        self.container.save_directory = self.save_directory
        self.operator = ao.AnnotationOperator()
        self.operator.generate_colorlist(len(self.container.id_dict))
        self.selected_label = self.label_list[1]
        self.ids.class_label_spinner.text = self.selected_label
        self.image_msgs = self.bag.read_messages(topics=[self.image_topic])
        self.operator.state = self.state
        self.guide_msg = self.operator.guide_msgs[self.state]
        for i in xrange(self.r_counter):
            self.image_msgs.next()
        self.readOneMsg()
        sm.current = 'Annotation'

    def readOneMsg(self):
        topic, msg, t = self.image_msgs.next()
        self.container.load_image_from_msg(msg)
        self.operator.draw_rect(self.container)
        self.ids.image_view.setImage(self.container.disp_image)

    def setState(self, val):
        prev_state = self.state
        self.operator.state = val
        self.state = self.operator.state
        self.guide_msg = self.operator.guide_msgs[self.state]

    def runCommand(self, val):
        self.container.finish_imageproc(save = (val == "norun"))
        self.w_counter = self.container.w_counter
        self.r_counter = self.container.r_counter
        self.readOneMsg()

    def setId(self, val):
        self.selected_label = val

    def setKeep(self, flag):
        self.container.keep_label = flag

class TouchTracer(Label, Image):
    def __init__(self, **kwargs):
        super(TouchTracer, self).__init__(**kwargs)
        Clock.schedule_interval(self.update, 1.0 / 60)
        self.image = cv2.imread("images/dummy.jpg")
        self.touch_pos = []
        self.touch_event = None

    def setImage(self, img):
            self.image = img

    def update(self, dt):
        image_buf = cv2.flip(self.image, 0)
        image_texture = Texture.create(size=(self.image.shape[1], self.image.shape[0]), colorfmt='bgr')
        image_texture.blit_buffer(image_buf.tostring(), colorfmt='bgr', bufferfmt='ubyte')
        self.texture = image_texture

    def calculate_touch_pos(self, touch):
        x = touch.x - self.pos[0]
        y = self.size[1] - touch.y + self.pos[1]
        stretch_ratio = self.width / self.image.shape[1]
        x = int(x / stretch_ratio);
        y = int(y / stretch_ratio);
        return [x, y]

    def in_area(self, pt):
        return (0 < pt[0] < self.image.shape[1] and 0 < pt[1] < self.image.shape[0])

    def on_touch_down(self, touch):
        pt = self.calculate_touch_pos(touch)
        if self.in_area(pt):
            self.touch_pos = pt
            self.touch_event = 0

    def on_touch_move(self, touch):
        pt = self.calculate_touch_pos(touch)
        if self.in_area(pt):
            self.touch_pos = pt
            self.touch_event = 1

    def on_touch_up(self, touch):
        pt = self.calculate_touch_pos(touch)
        if self.in_area(pt):
            self.touch_pos = pt
            self.touch_event = 2

class AnnotationApp(App):

    def __init__(self, **kwargs):
        super(AnnotationApp, self).__init__(**kwargs)
        self.title = 'Image Annotation'

    def build(self):
        return AnnotationWidget()#, TouchTracer()

    def on_pause(self):
        return True

if __name__ == '__main__':
    AnnotationApp().run()
