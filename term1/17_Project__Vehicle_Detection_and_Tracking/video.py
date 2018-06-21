import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
import cv2
import glob
import time
from sklearn.svm import LinearSVC
from sklearn.preprocessing import StandardScaler
from skimage.feature import hog
from mylib import *
import pickle
from moviepy.editor import VideoFileClip
#from IPython.display import HTML

_dist_pickle = None
_dist_pickle = pickle.load( open("svc_pickle.p", "rb" ) )
svc = _dist_pickle["svc"]
X_scaler = _dist_pickle["scaler"]
orient = _dist_pickle["orient"]
pix_per_cell = _dist_pickle["pix_per_cell"]
cell_per_block = _dist_pickle["cell_per_block"]
spatial_size = _dist_pickle["spatial_size"]
hist_bins = _dist_pickle["hist_bins"]
y_start_stop = _dist_pickle["y_start_stop"]
color_space = _dist_pickle["color_space"]

_tracker = Tracker()

def process_image(image):
    global a
    _merged_bboxes = CarDetection(image, y_start_stop, svc, X_scaler, orient, pix_per_cell, cell_per_block, spatial_size, hist_bins, color_space)
    out_img = draw_boxes(image, _merged_bboxes, color=(0, 0, 255), thick=6)
    if a<20:
        plt.imshow(out_img)
        plt.savefig('./output_images/aaa_' +str(a) +  '.jpg')
    a+=1
    return out_img

def CreateVideo(_mp4_file):
    clip1 = VideoFileClip(_mp4_file)
    _clip = clip1.fl_image(_tracker.next)
    _clip.write_videofile('output_images/'+_mp4_file, audio=False)

#CreateVideo('test_video.mp4')
CreateVideo('project_video.mp4')

