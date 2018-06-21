# 17. Project: Vehicle Detection and Tracking

## 1. List of files
* mylib.py : python program to define common functions
* train.py : python program to train SVM model.
* video.py : python program to detect cars on video files
* svc_pickle.p: pickle file including trained model
* output_images/test*_output.jpg : Examples of car detection results of static image frame.
* [test video](https://youtu.be/ix5Altzl1RY) : An example of car detection results of video data.
* [project video](https://youtu.be/jNDlbe2G8sc) : Video output of car detection result

[//]: # (Image References)
[test1]: ./output_images/test1_output.jpg
[test2]: ./output_images/test2_output.jpg
[test3]: ./output_images/test3_output.jpg
[test4]: ./output_images/test4_output.jpg
[test5]: ./output_images/test5_output.jpg
[test6]: ./output_images/test6_output.jpg
[car]: ./output_images/image0000.png
[noncar]: ./output_images/image6.png

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

### Histogram of Oriented Gradients (HOG)

#### 1. Explain how (and identify where in your code) you extracted HOG features from the training images.

The code for this step is contained in the function called 'extract_features()' of the file called 'mylib.py' and the function called 'train()' of the file called 'train.py'.  

I started by reading in all the `vehicle` and `non-vehicle` images from GTI and KITTI.
* Number of cars: 8792
* Number of non-cars: 8968

Here is an example of one of each of the `vehicle` and `non-vehicle` classes:

![car][car]
![non car][noncar]

I then explored different color spaces and different `skimage.hog()` parameters (`orientations`, `pixels_per_cell`, and `cells_per_block`).  I grabbed random images from each of the two classes and displayed them to get a feel for what the `skimage.hog()` output looks like.

#### 2. Explain how you settled on your final choice of HOG parameters.

I settled the following HOG parameters finally because of its good representation of car image. All of the 3-channels of hog features were used as well as spatial features and histgram features.
```python
color_space = 'YCrCb'
orient = 9
pix_per_cell = 8
cell_per_block = 2
spatial_size = (16, 16)
hist_bins = 16
hist_feat = True
hog_feat = True
y_start_stop = [400, None]
```


#### 3. Describe how (and identify where in your code) you trained a classifier using your selected HOG features (and color features if you used them).

The code for this step is contained in the function called 'train()' of the file called 'train.py'.

Data is splited into the following two data sets.
* Train/Vlidation data set: 14208
* Test set: 3552


I trained a linear SVM using inputs of 3-channel hog features, spatial features and histgram features. The feature vector length is 6108. The feature vector was normalized by using StandardScaler(). Lineaer SVM algorithm was used because of its better scalability to training data size than RBF kernel. I explored hyperparameter C of linear SVM model using grid search with 5-fold cross validation using 4:1 training/validation data set. After cross validation, and finally settled C=1 (default value of C of LinearSVC).


### Sliding Window Search

#### 1. Describe how (and identify where in your code) you implemented a sliding window search.  How did you decide what scales to search and how much to overlap windows?

I implemented a sliding window search in the function called 'find_cars()' in the file called 'mylib.py' (Hog Sub-sampling Window Search). If the scaling factor is equal to 1, the search window overlap is 75% because pix window consists of 8x8 cells and cells_per_step is equal to 2. I decided to search window positions all over the image at scales = 1, 1.5, 2, 3 to detect car at different sizes.

#### 2. Show some examples of test images to demonstrate how your pipeline is working.  What did you do to optimize the performance of your classifier?

I implemented a car detection pipeline in the function called 'CarDetection()' in the file called 'mylib.py'. I consolidated bounding boxes which were detected at different scales by using heatmap so that the final detection output of bounding box became stabler than a single scale.
Here are some example output images.

![test1][test1]
![test2][test2]
![test3][test3]
![test4][test4]
![test5][test5]
![test6][test6]


---

### Video Implementation

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (somewhat wobbly or unstable bounding boxes are ok as long as you are identifying the vehicles most of the time with minimal false positives.)
Here's a [link to my video result](https://youtu.be/jNDlbe2G8sc)

#### 2. Describe how (and identify where in your code) you implemented some kind of filter for false positives and some method for combining overlapping bounding boxes.

I implemented tracking algorithm in the class called 'Tracker()' in the file called 'mylib.py'. In this class, I recorded the positions of positive detections in each frame of the video.  From the positive detections I created a heatmap and then thresholded that map to identify vehicle positions.  I then used `scipy.ndimage.measurements.label()` to identify individual blobs in the heatmap.  I then assumed each blob corresponded to a vehicle.  I constructed bounding boxes to cover the area of each blob detected.  

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

Even after implementing the tracker algorithm, the sizes and positions of bboxes are not stable enoughg. Further, there are some periods of false negatives of white car around 0m05s and 0m28s in the video. There might be more rooms to tune SVM model and HOG parameters to gain better performance and lower false negatives. In addition, the performance could be much better if I used CNN instead of SVM for classification, or if I used Faster-RCNN or Yolo as detection algorithm.
