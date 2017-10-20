import csv
import cv2
import numpy as np
from keras.models import Sequential
from keras.layers.core import Lambda, Flatten, Dense, Dropout, Activation
from keras.layers.convolutional import Convolution2D, Cropping2D
from keras.layers.pooling import MaxPooling2D
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle
from config import *
import random

lines = []
with open('./driving_log.csv') as f:
    reader = csv.reader(f)
    for line in reader:
        lines.append(line)

training_samples, validaction_samples = train_test_split(lines[1:], test_size=0.2)

def generator(samples, bias=0.5, batch_size=CONFIG['batch_size']):
    """
    Perform preprosessing and on single BGR frame, decrease bias toward zero, add small variance to steering angle,
    randomly choose right, left and center frame then flip the image(50% chance)  .

    Parameters
    ----------
    samples : splited data_log, each row containing file path and steering angle
    bias : parameters to combat steering angle unbalance (toward 0)
    batch_size : number of training data for each batch

    yield
    -------
    X_train : batch_size of preprocessed image data.
    y_train : batch_size of propressed steering angle data.
    """
    correction = CONFIG['correction']
    num_samples = len(samples)
    while 1:
        samples = shuffle(samples)
        X_train = np.zeros(shape=(batch_size, 160, 320, 3), dtype=np.float32)
        y_train = np.zeros(shape=(batch_size,), dtype=np.float32)
        img_count = 0
        for batch_sample in samples:
            center_angle = float(batch_sample[3])
            if abs(center_angle) + bias < np.random.rand():
                continue # for training balance
            index = random.choice([0,1,2]) # randomly select front, left or right
            if index == 0:
                angle = center_angle
            elif index == 1:
                angle = center_angle + correction
            else:
                angle = center_angle - correction
            img_name = './IMG/' + batch_sample[index].split('/')[-1]
            img = cv2.imread(img_name)
            if random.choice([True, False]):
                img = cv2.flip(img,1)
                angle = -angle
            angle += np.random.normal(loc=0, scale=CONFIG['steer_sigma'])

            X_train[img_count] = img
            y_train[img_count] = angle
            img_count += 1
            if img_count == batch_size:
                yield X_train, y_train
                break




train_genterator = generator(training_samples, bias=0.5)
validation_generator = generator(validaction_samples, bias=1.)

init = 'glorot_uniform'

model = Sequential()
model.add(Lambda(lambda x: x / 255.0 - 0.5, input_shape=(160,320,3)))
model.add(Cropping2D(cropping=((70,25), (0,0))))
model.add(Convolution2D(24,5,5, subsample=(2,2), activation='relu', init=init))
model.add(Dropout(0.2))
model.add(Convolution2D(36,5,5, subsample=(2,2), activation='relu', init=init))
model.add(Dropout(0.2))
model.add(Convolution2D(48,5,5, subsample=(2,2), activation='relu', init=init))
model.add(Dropout(0.2))
model.add(Convolution2D(64,3,3, activation='relu', init=init))
model.add(Dropout(0.2))
model.add(Convolution2D(64,3,3, activation='relu', init=init))
model.add(Dropout(0.2))
model.add(Flatten())
model.add(Dense(100, init=init))
model.add(Activation('relu'))
model.add(Dropout(0.5))
model.add(Dense(50, init=init))
model.add(Activation('relu'))
model.add(Dropout(0.5))
model.add(Dense(10, init=init))
model.add(Activation('relu'))
model.add(Dense(1, init=init))

model.compile(loss='mse', optimizer='adam') # Mean square error, since it is a regression problem
model.fit_generator(train_genterator, samples_per_epoch=len(training_samples), validation_data=validation_generator,
                    nb_val_samples=len(validaction_samples), nb_epoch=10)

model.save('model.h5')