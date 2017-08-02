# Image annotation tools #

Tools for generating iamge dataset for digits from rosbag.

TBD: caffe and chainer style dataset generating

---

## 1. Image Annotation ##

### 1-1. GUI Annotation Tool ###

Usage:  
```
$ ./image_annotation.py
```

Then you can input parameters as the image below:
*image : TBD*

Click 'Start' button to start annotation.

### 1-1. GUI Annotation Tool ###

Usage:  
1. Revise image_topic and save_directory in cui_image_annotation.py(L17,18). save_directory must contain two child directories named "images" and "labels".  
2. Write class list text file. Sample is class_list.txt  
3. Run image_annotation as below.
```
$ ./cui_image_annotatin.py <bagfile_name> <class_list>
```

You can see detailed usage for labelling by type 'h' after start running image_annotation.

---

## 2. Check annotation ##

You can check annotation result as:  
```
$ ./check_annotation.py <data_directory> <class_list>
```

data_directory is same as save_directory you defined for annotation.
