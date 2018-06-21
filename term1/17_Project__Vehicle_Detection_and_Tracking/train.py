import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
import cv2
import glob
import time
from sklearn.svm import LinearSVC
from sklearn.preprocessing import StandardScaler
from skimage.feature import hog
import pickle
from mylib import *
from sklearn.grid_search import GridSearchCV

# NOTE: the next import is only valid for scikit-learn version <= 0.17
# for scikit-learn >= 0.18 use:
# from sklearn.model_selection import train_test_split
from sklearn.cross_validation import train_test_split

def train():
    # Read in cars and notcars
    cars    = glob.glob('./data/vehicles/*/*.png')
    notcars = glob.glob('./data/non-vehicles/*/*.png')

    # Reduce the sample size because
    # The quiz evaluator times out after 13s of CPU time
    #sample_size = 500
    sample_size = 10000
    cars = cars[0:sample_size]
    notcars = notcars[0:sample_size]

    ### TODO: Tweak these parameters and see how the results change.
    color_space = 'YCrCb' # Can be RGB, HSV, LUV, HLS, YUV, YCrCb
    orient = 9  # HOG orientations
    pix_per_cell = 8 # HOG pixels per cell
    cell_per_block = 2 # HOG cells per block
    spatial_size = (16, 16) # Spatial binning dimensions
    hist_bins = 16    # Number of histogram bins
    hist_feat = True # Histogram features on or off
    hog_feat = True # HOG features on or off
    y_start_stop = [400, None] # Min and max in y to search in slide_window()

    car_features = extract_features(cars, color_space=color_space, 
                        spatial_size=spatial_size, hist_bins=hist_bins, 
                        orient=orient, pix_per_cell=pix_per_cell, 
                        cell_per_block=cell_per_block)
    notcar_features = extract_features(notcars, color_space=color_space, 
                        spatial_size=spatial_size, hist_bins=hist_bins, 
                        orient=orient, pix_per_cell=pix_per_cell, 
                        cell_per_block=cell_per_block)

    X = np.vstack((car_features, notcar_features)).astype(np.float64)                        
    # Fit a per-column scaler
    X_scaler = StandardScaler().fit(X)
    # Apply the scaler to X
    scaled_X = X_scaler.transform(X)

    # Define the labels vector
    y = np.hstack((np.ones(len(car_features)), np.zeros(len(notcar_features))))

    # Split up data into randomized training and test sets
    rand_state = np.random.randint(0, 100)
    X_train, X_test, y_train, y_test = train_test_split(scaled_X, y, test_size=0.2, random_state=rand_state)

    print('Using:',orient,'orientations',pix_per_cell, 'pixels per cell and', cell_per_block,'cells per block')
    print('Feature vector length:', len(X_train[0]))
    print('Train size:', len(X_train))
    print('Test size:', len(X_test))
    svc = LinearSVC()
    param_grid = {'C': [0.5, 1, 1.5, 10, 100], 'penalty':['l2']}
    clf = GridSearchCV(svc, param_grid, cv=5, scoring='accuracy', verbose=1)
    clf.fit(X_train, y_train)
    _best_C = clf.best_params_['C']
    print(clf.best_params_, clf.best_score_)
    print(clf.grid_scores_)

    svc = LinearSVC(C=_best_C, verbose=True)
    t=time.time()
    svc.fit(X_train, y_train)
    t2 = time.time()
    print(round(t2-t, 2), 'Seconds to train SVC...')
    print('Test Accuracy of SVC = ', round(svc.score(X_test, y_test), 4))

    _dist_pickle = {}
    _dist_pickle["svc"] = svc
    _dist_pickle["scaler"] = X_scaler
    _dist_pickle["orient"] = orient
    _dist_pickle["pix_per_cell"] = pix_per_cell
    _dist_pickle["cell_per_block"] = cell_per_block
    _dist_pickle["spatial_size"] = spatial_size
    _dist_pickle["hist_bins"] = hist_bins
    _dist_pickle["y_start_stop"] = y_start_stop
    _dist_pickle["color_space"] = color_space
    with open('svc_pickle.p', mode='wb') as _f:
        pickle.dump(_dist_pickle, _f)

train()
###################################################
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

_list_image_files = ['./test_images/test1.jpg', './test_images/test2.jpg', './test_images/test3.jpg', './test_images/test4.jpg', './test_images/test5.jpg', './test_images/test6.jpg']
for _image_file in _list_image_files:
    image = cv2.imread(_image_file)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    _merged_bboxes = CarDetection(image, y_start_stop, svc, X_scaler, orient, pix_per_cell, cell_per_block, spatial_size, hist_bins, color_space)
    out_img = draw_boxes(image, _merged_bboxes, color=(0, 0, 255), thick=6)
    plt.imshow(out_img)
    plt.title(_image_file)
    plt.savefig('./output_images/' + _image_file.split('/')[-1].split('.')[0]+ '_output.jpg')
