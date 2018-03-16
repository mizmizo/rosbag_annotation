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
from kivy.core.window import Window

random.seed(5)

class AnnotationWidget(Screen, Label, Image):
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
        self.label_list = []
        self.guide_msg = ""
        self.container = None
        self.operator = None
        self.keyboard = None
        self.touch_pos = []
        self.touch_event = None # 0: down, 1: move, 2:up, 3:wait

    def update(self, dt):
        if (self.touch_pos != self.ids.image_view.touch_pos or
            self.touch_event != self.ids.image_view.touch_event):
            self.touch_pos = self.ids.image_view.touch_pos
            self.touch_event = self.ids.image_view.touch_event
            self.operator.click_annotate_rect(self.touch_event,
                                              self.touch_pos[0], self.touch_pos[1],
                                              self.container, self.selected_label)
            self.screenupdate()

    def screenupdate(self):
        self.ids.image_view.setImage(self.container.disp_image)
        self.selected_label = self.operator.label


    def startAnnotation(self):
        # setup image data : from rosbag
        if not bool(self.ids.from_rosbag.collapse):
            data_type = 0 ## 0: rosbag, 1: annotated
            # bagfile
            try:
                bag = rosbag.Bag(self.ids.bagfile_path.text, 'r')
            except Exception as e:
                self.guide_msg = "Bag File Error!"
                return
            # image topic
            if self.ids.topic_name.text == "":
                self.guide_msg = "Topic name is Empty!"
                return
            else:
                image_topic = self.ids.topic_name.text
        else:
            # setup image data : from annotated data
            data_type = 1
            if self.ids.annotated_path.text == "":
                self.guide_msg = "Input directory is Empty!"
                return
            else:
                annotated_directory = self.ids.annotated_path.text
                if not os.path.isdir(annotated_directory):
                    self.guide_msg = "Invalid input directory name!"
                    return
                if (not os.path.isdir(annotated_directory + "/images/") or
                    not os.path.isdir(annotated_directory + "/labels/")):
                    self.guide_msg = "Input directory must have 'images' and 'labels' subdirectory."
                    return
                annotated_images = sorted(os.listdir(annotated_directory + "/images/"),
                                          reverse=True)
                annotated_labels = sorted(os.listdir(annotated_directory + "/labels/"),
                                          reverse=True)
                if len(annotated_images) != len(annotated_labels):
                    self.guide_msg = "Images and labels number is different!"
                    return


        # class list
        if self.ids.class_list.text == "":
            self.guide_msg = "Class list is Empty!"
            return
        else:
            class_path = self.ids.class_list.text

        # save directory
        if self.ids.save_path.text == "":
            self.guide_msg = "Save directory is Empty!"
            return
        else:
            save_directory = self.ids.save_path.text
        if not os.path.isdir(save_directory):
            self.guide_msg = "Invalid save directory name!"
            return
        # start writing counter
        try:
            self.w_counter = int(self.ids.start_w_counter.text)
        except Exception as e:
            self.guide_msg = "Enter integer in start counter!"
            return

        # start reading counter
        try:
            self.r_counter = int(self.ids.start_r_counter.text)
        except Exception as e:
            self.guide_msg = "Enter integer in start counter!"
            return

        ## preparation

        # init input data
        if data_type == 0:
            # init rosbag reader and pop to start position
            image_msgs = bag.read_messages(topics=[image_topic])
            for i in xrange(self.r_counter):
                image_msgs.next()
        elif data_type == 1:
            # init data reader and pop to start position
            for i in xrange(self.r_counter):
                annotated_images.pop()
                annotated_labels.pop()

        # init container and operator
        self.container = ac.AnnotationContainer(save_directory, self.w_counter, True)
        self.container.register_dict(class_path)
        for x, name in sorted(self.container.id_dict.items()):
            self.label_list.append(name)
        if data_type == 0:
            self.operator = ao.AnnotationOperator(data_type, image_msgs)
        else:
            self.operator = ao.AnnotationOperator(data_type, (annotated_directory,
                                                              annotated_images,
                                                              annotated_labels))
        self.operator.generate_colorlist(len(self.container.id_dict))
        self.setState('addanno')

        # init spinner and guide message
        self.setLabel(self.label_list[1])
        # init keyboard input
        self.keyboard = Window.request_keyboard(self.keyboardShutdown, self)
        self.keyboard.bind(on_key_down=self.keyboardCallback)
        self.readDataOnce()
        self.ids.smanager.current = 'Annotation'


    def readDataOnce(self):
        self.operator.pop_data(self.container)
        self.operator.draw_rect(self.container)
        self.screenupdate()

    def setState(self, val):
        self.operator.set_state(val)
        self.selected_label = self.operator.label
        self.guide_msg = self.operator.guide_msgs[self.operator.state]


    def setLabel(self, val):
        self.operator.set_label(val, self.container)
        self.selected_label = self.operator.label
        self.screenupdate()


    def runCommand(self, val):
        self.operator.finish_proc(val, self.container)
        self.w_counter = self.container.w_counter
        self.r_counter = self.container.r_counter
        self.readDataOnce()


    def setKeep(self, flag):
        self.operator.set_keep(flag)

    def keyboardShutdown(self):
        self.keyboard.unbind(on_key_down=self.keyboardCallback)
        self.keyboard = None

    def keyboardCallback(self, keyboard, keycode, text, modifiers):
        if text.isdigit():
            if int(text) < len(self.label_list):
                self.setLabel(self.label_list[int(text)])
                self.screenupdate()
        elif text == 's':
            self.runCommand('norun')
            print("save")
        elif text == 'n':
            self.runCommand('skip')
            print("skip")

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
