import numpy as np
import sklearn
from sklearn.model_selection import train_test_split
import cv2
import os
import csv
import random
import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt
from keras.models import Sequential, Model
from keras.layers import Flatten, Dense, Lambda, Permute, Dropout, Cropping2D, BatchNormalization, Activation
from keras.layers.convolutional import Convolution2D
from keras.layers import MaxPooling2D


# Model Drivers
_dirs = ['./data/0_sample/', './data/9_model_driver/', './data/A_model_driver/']
_samples = []
for _dir in _dirs:
    print(_dir)
    with open(_dir+'driving_log.csv') as _csvfile:
        _reader = csv.reader(_csvfile)
        for _line in _reader:
            if len(_line)>0 and _line[0]!='center':
                _line[0] = _dir+'IMG/'+_line[0].split('/')[-1].split('\\')[-1]
                _line[1] = _dir+'IMG/'+_line[1].split('/')[-1].split('\\')[-1]
                _line[2] = _dir+'IMG/'+_line[2].split('/')[-1].split('\\')[-1]
                _samples.append(_line)

# Adjustments
_dirs = ['./data/1_mydata/', './data/2_mydata/', './data/4_mydata/', './data/5_mydata/', './data/6_mydata/', './data/7_mydata/','./data/8_adjustment/' ]
for _dir in _dirs:
    print(_dir)
    with open(_dir+'driving_log.csv') as _csvfile:
        _reader = csv.reader(_csvfile)
        for _line in _reader:
            if len(_line)>0 and _line[0]!='center':
                _line[0] = _dir+'IMG/'+_line[0].split('/')[-1].split('\\')[-1]
                _line[1] = _dir+'IMG/'+_line[1].split('/')[-1].split('\\')[-1]
                _line[2] = _dir+'IMG/'+_line[2].split('/')[-1].split('\\')[-1]
                if float(_line[3])>0.01:
                    _samples.append(_line)
                #else:
                #    if random.randint(0,10)==0:
                #        _samples.append(_line)

_train_samples, _validation_samples = train_test_split(_samples, test_size=0.2)

def generator(_samples, _batch_size=32):
    _num_samples = len(_samples)
    while 1:
        random.shuffle(_samples)
        for _offset in range(0, _num_samples, _batch_size):
            _batch_samples = _samples[_offset:_offset+_batch_size]
            _images = []
            _angles = []
            for _batch_sample in _batch_samples:
                _center_image = cv2.imread(_batch_sample[0])
                _left_image = cv2.imread(_batch_sample[1])
                _right_image = cv2.imread(_batch_sample[2])
                _center_angle = float(_batch_sample[3])
                _left_angle = _center_angle+.2
                _right_angle = _center_angle-.2

                #if True:
                if random.randint(0,2)==0:
                    _image = _center_image
                    _angle = _center_angle
                elif random.randint(0,2)==1:
                    _image = _left_image
                    _angle = _left_angle
                else:
                    _image = _right_image
                    _angle = _right_angle
                    
                if random.randint(0,1)==0:
                    _images.append(_image)
                    _angles.append(_angle)
                else:
                    _images.append(cv2.flip(_image, 1))
                    _angles.append(-_angle)

            _X_train = np.array(_images)
            _y_train = np.array(_angles)
            yield sklearn.utils.shuffle(_X_train, _y_train)

_train_generator = generator(_train_samples, _batch_size=32)
_validation_generator = generator(_validation_samples, _batch_size=32)

_ch, _row, _col = 3, 160, 320

_model = Sequential()
_model.add(Lambda(lambda _x: _x/255.0-0.5, input_shape=(_row, _col, _ch), output_shape=(_row, _col, _ch)))
_model.add(Cropping2D(cropping=((50,20),(0,0))))
_model.add(Convolution2D(16, 3,3,activation="relu"))
#_model.add(Convolution2D(16, 3,3))
#_model.add(BatchNormalization())
#_model.add(Activation('relu'))
_model.add(Convolution2D(16, 3,3,activation="relu"))
#_model.add(Convolution2D(16, 3,3))
#_model.add(BatchNormalization())
#_model.add(Activation('relu'))
_model.add(MaxPooling2D(pool_size=(2,2)))
_model.add(Dropout(0.25))
_model.add(Flatten())
_model.add(Dense(64))
_model.add(Dropout(0.5))
_model.add(Dense(1))

_model.compile(loss='mse', optimizer='adam')
print(_model.summary())
_fit_history = _model.fit_generator(_train_generator, samples_per_epoch=len(_train_samples), validation_data=_validation_generator, nb_val_samples=len(_validation_samples), nb_epoch=10)
_model.save('model.h5')

print(_fit_history.history.keys())

plt.plot(_fit_history.history['loss'])
plt.plot(_fit_history.history['val_loss'])
plt.title('model mean squared error loss')
plt.ylabel('mean squared error loss')
plt.xlabel('epoch')
plt.legend(['training set', 'validation set'], loc='upper right')
plt.savefig('./model.png')
