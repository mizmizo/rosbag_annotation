#!/usr/bin/env python

from __future__ import print_function

import numpy as np
import cv2
import sys
import os
from scipy import misc
import random
import time
import annotation_container as ac

import imgaug as ia
from imgaug import augmenters as iaa
from sklearn.utils import shuffle

start_counter = 0 ## Start saving label from this number
visualize = False
aug_size = 7

window_name = 'image_augmentation'
random.seed(5)
color_list = []
show_time = 100 # show each image and label for show_time ms

## <label_string 0.0 0 0.0 x_min y_min x_max y_max 0.0 0.0 0.0 0.0 0.0 0.0 0.0>

def data_augmentation(data_directory, save_directory, class_path):
    # generate container
    container = ac.AnnotationContainer(save_directory, 0, False)

    ## register class list
    container.register_dict(class_path)
    container.show_dict()

    ## generate color list
    rand_list = [random.randint(0, 255) for i in xrange(len(container.id_dict) * 3)]
    for i in xrange(len(container.id_dict)):
        color_list.append(rand_list[i * 3:(i + 1) * 3])

    if visualize:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    ## get image and label name list and shuffle them
    image_list = sorted(os.listdir(data_directory + "/images/"))
    label_list = sorted(os.listdir(data_directory + "/labels/"))
    image_list, label_list = shuffle(image_list, label_list)

    print("Start augmentation, type 'q' for quiting.")

    ## load label and visualize
    for image_name, label_name in zip(image_list, label_list):

        print("processing {0}...".format(image_name))
        # read image and label
        images = []
        image = cv2.imread(data_directory + "/images/" + image_name, -1)
        rects = [] # left-top and right-bottom points
        rois = [] # all vertices
        labels = []
        for line in open(data_directory + "/labels/" + label_name, "r"):
            line = line.rstrip("\n")
            labels.append(line.split()[0])
            rect_str = line.split()[4:8]
            vertices = []
            for i in xrange(len(rect_str)):
                vertices.append(int(float(rect_str[i])))
            rect = [(vertices[0], vertices[1]), (vertices[2], vertices[3])]
            roi = np.array([[vertices[0], vertices[1], 1],
                            [vertices[0], vertices[3], 1],
                            [vertices[2], vertices[3], 1],
                            [vertices[2], vertices[1], 1]])
            rects.append(rect)
            rois.append(roi)

        # set keypoints
        keypoints_on_images = []
        for _ in xrange(aug_size):
            keypoints = []
            roi = rects[0]
            for i in xrange(4):
                point = []
                if i < 2:
                    point.append(roi[0][0])
                else:
                    point.append(roi[1][0])
                if i in (0, 3):
                    point.append(roi[0][1])
                else:
                    point.append(roi[1][1])
                keypoints.append(ia.Keypoint(x=point[0], y=point[1]))
            images.append(image)
            keypoints_on_images.append(ia.KeypointsOnImage(keypoints, shape=images[0].shape))

        # set augmentation type
        # ===== Revise here to chenge augmentation ===== #
        sometimes = lambda aug: iaa.Sometimes(0.5, aug)
        seq = iaa.Sequential([sometimes(iaa.GaussianBlur((0, 2.0))),
                              #iaa.Crop(percent=(0, 0.2)),
                              iaa.Affine(
                                  scale={"x": (0.8, 1.2), "y": (0.8, 1.2)},
                                  translate_percent={"x": (-0.1, 0.1), "y": (-0.1, 0.1)},
                                  rotate=(-5, 5),
                                  shear=(-8, 8),
                                  ),
                              iaa.Add((-20, 20)),
                              iaa.ContrastNormalization((0.7, 1.3))])
        # ============================================== #
        seq_det = seq.to_deterministic()

        # augment
        container.class_vec = list(labels)
        container.rect_vec = list(rects)
        images_aug = seq_det.augment_images(images)
        keypoints_aug = seq_det.augment_keypoints(keypoints_on_images)

        # save original image
        container.save_image = images[0]
        container.finish_imageproc()

        # update label
        for img_idx, (image_before, image_after, keypoints_before, keypoints_after) in enumerate(zip(images, images_aug, keypoints_on_images, keypoints_aug)):
            keypoints_old = np.empty((0, 3))
            keypoints_new = np.empty((0, 3))
            image_rected_after = image_after.copy()
            # image_before = keypoints_before.draw_on_image(image_before)
            # image_after = keypoints_after.draw_on_image(image_after)
            for kp_idx, keypoint in enumerate(keypoints_after.keypoints):
                keypoint_old = keypoints_on_images[img_idx].keypoints[kp_idx]
                x_old, y_old = keypoint_old.x, keypoint_old.y
                x_new, y_new = keypoint.x, keypoint.y
                keypoints_old = np.append(keypoints_old, np.array([[x_old, y_old, 1]]), axis=0)
                keypoints_new = np.append(keypoints_new, np.array([[x_new, y_new, 1]]), axis=0)

            pinv_keypoints_old = np.linalg.pinv(keypoints_old.transpose())
            affine = keypoints_new.transpose().dot(pinv_keypoints_old)
            rects_new = []
            for i, roi in enumerate(rois):
                roi_new = np.dot(affine, roi.transpose())
                rect_new = [(int(max(0, min(roi_new[0]))), int(max(0, min(roi_new[1])))),
                            (int(min(image_after.shape[1],  max(roi_new[0]))), int(min(image_after.shape[0], max(roi_new[1]))))]
                rects_new.append(rect_new)

                # check new label
                # out of image
                if rect_new[0][0] > image_after.shape[1] or rect_new[0][1] > image_after.shape[0] or rect_new[1][0] < 0 or rect_new[1][1] < 0:
                    continue

                cv2.rectangle(image_before, rects[i][0], rects[i][1], color_list[int(container.id_reverse_dict[labels[i]])], 2)
                cv2.putText(image_before, labels[i], (rects[i][0][0], rects[i][0][1] - 5), cv2.FONT_HERSHEY_TRIPLEX, 1.0, color_list[int(container.id_reverse_dict[labels[i]])])

                cv2.rectangle(image_rected_after, rect_new[0], rect_new[1], color_list[int(container.id_reverse_dict[labels[i]])], 2)
                cv2.putText(image_rected_after, labels[i], (rect_new[0][0], rect_new[0][1] - 5), cv2.FONT_HERSHEY_TRIPLEX, 1.0, color_list[int(container.id_reverse_dict[labels[i]])])
                container.class_vec.append(labels[i])
                container.rect_vec.append(rect_new)

            # save new label and image
            container.save_image = image_after
            container.finish_imageproc()
            if visualize:
                cv2.imshow(window_name, np.concatenate((image_before, image_rected_after), axis=1))
                key = cv2.waitKey(show_time) & 0xFF
                if chr(key) == 'q':
                    return

def main(argv):
    if len(argv) < 4:
        print('provide path to data, output directory and class list')
        return

    print('\033[33mchecking directory\033[0m')
    try:
        isdir_1 = os.path.isdir(argv[1])
        isdir_2 = os.path.isdir(argv[2])
        if not isdir_1 or not isdir_2:
            raise ValueError('\033[31mdata directory not found\033[0m')
    except ValueError, e:
        print(e)
        raise SystemExit

    data_augmentation(argv[1], argv[2], argv[3])

if __name__ == "__main__":
    main(sys.argv)
