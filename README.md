# Rosbag annotation #

Annotation tool for generating iamge dataset for digits and chainercv from rosbag.

## 1. Requirements ##

- ROS(indigo or newer)
- OpenCV(only checked on version 3.1.0)

## 2. Installation ##

### 2-1. annotation tool setup ###

install.sh will install all requirements.
```
$ ./install.sh
```

- [imgaug](https://github.com/aleju/imgaug)
- [kivy](https://kivy.org/docs/installation/installation-linux.html)

**CAUTION**

install.sh installs cython(0.25.2) and pygame from pip.
Install imgaug and kivy manually if you are using different version for cython.

### 2-2. setup chainer dataset ###

---

## 3. Image Annotation ##

### 2-1. GUI Annotation Tool ###

![annotation](images/annotation.png)

Usage:  
```
$ ./rosbag_annotation.py
```

Then you can input parameters as the image below:
![setup](images/setup.png)

Parameters are:  
- Rosbag File : source rosbag file full path which contains Sensor_msgs/Image topic data.  
- Topic of Interest : Name of the image topic  
- Class List : Full path to the text file which contains a list of classes. Sample is class_list.txt  
- Save Directory : Full path to the directory to save annotated dataset.  
- Start writing counter : Number of topic reading iterations before annotation  
- Start reading counter : This tool will start saving data with this number.

Click 'Start' button to start annotation.

Annotated dataset will be saved in <Save Directory>/images/ and <Save Directory>/labels/.  
Those data have digits detection format (same as KITTI except score).
You can use the dataset from chainer with detection_dataset.py.

### 2-2. CUI Annotation Tool (deprecated) ###

Usage:  
1. Revise image_topic and save_directory in cui_image_annotation.py(L17,18).  
2. Write class list text file. Sample is class_list.txt  
3. Run image_annotation as below.  
4. There will be two directories named images and labels in save_directory where images contains images, labels contains annotation labels.
```
$ ./cui_image_annotatin.py <bagfile_name> <class_list>
```

You can see detailed usage for labelling by type 'h' after start running image_annotation.

---

## 3. Data augmentation ##

```
$ ./image_augmentation.py <data_directory> <save_directory> <class_list>
```

---

## 4. Split training data and validation data ##

```
$ ./split_train_val.py <data_directory> <save_directory>
```

## 5. Check annotation ##

You can check annotation result as:  
```
$ ./check_annotation.py <data_directory> <class_list>
```

data_directory is same as save_directory you defined for annotation.
