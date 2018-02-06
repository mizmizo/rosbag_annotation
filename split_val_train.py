#!/usr/bin/env python

from __future__ import print_function

import os
import sys
import shutil
import random

# param
val_ratio = 0.1 # default: split 10% of original to validation
keep_original = False # delete original file when False

def data_split(data_directory, save_directory):
    # check directory and setup
    for splitname in ("/train/", "/val/"):
        if not os.path.isdir(save_directory + splitname):
            os.mkdir(save_directory + splitname)
        for dirname in ("images/", "labels/"):
            if not os.path.isdir(save_directory + splitname + dirname):
                os.mkdir(save_directory + splitname + dirname)

    image_list = sorted(os.listdir(data_directory + "/images/"))
    label_list = sorted(os.listdir(data_directory + "/labels/"))

    for image, label in zip(image_list, label_list):
        if random.uniform(0, 1) < val_ratio:
            if keep_original:
                shutil.copy(data_directory + "/images/" + image, save_directory + "/val/images/" + image)
                shutil.copy(data_directory + "/labels/" + label, save_directory + "/val/labels/" + label)
            else:
                shutil.move(data_directory + "/images/" + image, save_directory + "/val/images/" + image)
                shutil.move(data_directory + "/labels/" + label, save_directory + "/val/labels/" + label)
        else:
            if keep_original:
                shutil.copy(data_directory + "/images/" + image, save_directory + "/train/images/" + image)
                shutil.copy(data_directory + "/labels/" + label, save_directory + "/train/labels/" + label)
            else:
                shutil.move(data_directory + "/images/" + image, save_directory + "/train/images/" + image)
                shutil.move(data_directory + "/labels/" + label, save_directory + "/train/labels/" + label)


def main(argv):
    if len(argv) < 3:
        print('provide path to data and output directory')
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

    data_split(argv[1], argv[2])

if __name__ == "__main__":
    main(sys.argv)

