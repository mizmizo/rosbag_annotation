# Image annotation tools #

Tools for generating learning dataset for caffe from rosbag.

---

## 1. Image Annotation ##

Usage:  
1. Revise image_topic and save_directory in image_annotation.py(L18,19). save_directory must contain two child directories named "images" and "labels".
2. Write class list text file. Sample is class_list.txt
3. Run image_annotation as below.
```
$ ./image_annotatin.py <bagfile_name> <class_list>
```

You can see detailed usage for labelling by type 'h' after start running image_annotation.

---

## 2. Check annotation ##

You can check annotation result as:  
```
$ ./check_annotation.py <data_directory> <class_list>
```

data_directory is same as save_directory you defined for annotation.
