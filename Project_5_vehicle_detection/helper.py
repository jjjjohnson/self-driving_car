import numpy as np
import cv2
import glob
import matplotlib.image as mpimg
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
from sklearn.svm import LinearSVC
from skimage.feature import hog
from sklearn.calibration import CalibratedClassifierCV
import pickle

ystart = 400
ystop = 656
scale = 1.5
spatial_size = (32,32)
hist_bins = 32
orient = 9
pix_per_cell = 8
cell_per_block = 2

def convert_color(img, conv='RGB2YCrCb'):
    if conv == 'RGB2YCrCb':
        return cv2.cvtColor(img, cv2.COLOR_RGB2YCrCb)
    if conv == 'BGR2YCrCb':
        return cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)
    if conv == 'RGB2LUV':
        return cv2.cvtColor(img, cv2.COLOR_RGB2LUV)
    if conv == 'BGR2HSV':
        return cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    if conv == 'RGB2HSV':
        return cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

def bin_spatial(img, size=(32,32)):
    color1 = cv2.resize(img[:,:,0], size).ravel()
    color2 = cv2.resize(img[:,:,1], size).ravel()
    color3 = cv2.resize(img[:,:,2], size).ravel()
    spatial_features = np.hstack((color1, color2, color3))
    return spatial_features

def color_hist(image, nbins=32):
    # Compute the histogram of the color channels seperately. Ranges for HSV
    channel1_hist = np.histogram(image[:,:,0],nbins,range=(0,179)) # hist[0] -> counts in each bin
    channel2_hist = np.histogram(image[:,:,1],nbins,range=(0,255)) # hist[1] -> bin edges
    channel3_hist = np.histogram(image[:,:,2],nbins,range=(0,255))
    # Concatenate the histograms into a single feature vector
    hist_features = np.concatenate((channel1_hist[0],channel2_hist[0],channel3_hist[0]))
    return hist_features

def get_hog_features(img, orient, pix_per_cell, cell_per_block, vis=False, feature_vec=True):
    if vis == True:
        features, hog_image = hog(img, orient, (pix_per_cell,pix_per_cell), (cell_per_block,cell_per_block), visualise=vis, feature_vector=feature_vec)
        return features, hog_image
    else:
        features = hog(img, orient, (pix_per_cell,pix_per_cell), (cell_per_block,cell_per_block), visualise=vis, feature_vector=feature_vec)
        return features

def extract_features(imgs,orient, pix_per_cell, cell_per_block, spatial_size, hist_bins):
    """Read the training images, extract features. Return concatenated features"""
    features = []
    for file in imgs:
        # Read in each one by one
        # if '.png' in file:
        #     image = mpimg.imread(file) * 255
        # else:
        image = cv2.imread(file)
        image = convert_color(image, conv='RGB2HSV')

        ch1 = image[:,:,0]
        ch2 = image[:,:,1]
        ch3 = image[:,:,2]
        # Compute individual channel HOG features for the entire image
        hog1 = get_hog_features(ch1, orient, pix_per_cell, cell_per_block, feature_vec=True)
        hog2 = get_hog_features(ch2, orient, pix_per_cell, cell_per_block, feature_vec=True)
        hog3 = get_hog_features(ch3, orient, pix_per_cell, cell_per_block, feature_vec=True)
        hog_features = np.hstack((hog1, hog2, hog3))
        # Get color features
        spatial_features = bin_spatial(image, size=spatial_size)
        hist_features = color_hist(image, nbins=hist_bins)

        # Concatanated all features
        all_features = np.hstack((spatial_features, hist_features, hog_features))
        features.append(all_features)
    return features

if __name__ == '__main__':
    non_vehicles = glob.glob('/Users/jinjunjie/a_self_driving_car/Vehicle detection and tracking/images/non-vehicles/*/*.png')
    vehicles = glob.glob('/Users/jinjunjie/a_self_driving_car/Vehicle detection and tracking/images/vehicles/*/*.png')

    non_vehicles_features = extract_features(non_vehicles,orient=orient, pix_per_cell=pix_per_cell,
                                             cell_per_block=cell_per_block,spatial_size=spatial_size, hist_bins=hist_bins)
    vehicles_features = extract_features(vehicles,orient=orient, pix_per_cell=pix_per_cell,
                                         cell_per_block=cell_per_block,spatial_size=spatial_size, hist_bins=hist_bins)

    # Create an array stack of feature vectors
    X = np.vstack((vehicles_features, non_vehicles_features)).astype(np.float64)
    # Fit a per-column scaler
    X_scaler = StandardScaler().fit(X)
    # Apply the scaler to X
    scaled_X = X_scaler.transform(X)

    # Define the labels vector
    y = np.hstack((np.ones(len(vehicles_features)), np.zeros(len(non_vehicles_features))))

    # Split up data into randomized training and test sets
    X_train, X_test, y_train, y_test = train_test_split(scaled_X, y, test_size=0.2, random_state=0)

    # Use a linear SVC
    svc = CalibratedClassifierCV(LinearSVC())

    svc.fit(X_train, y_train)

    # save the model to disk. Why does not work in ipython notebook???
    with open('svc.sav', 'wb') as f:
        pickle.dump(svc, f)

    with open('X_scaler.sav','wb') as f:
        pickle.dump(X_scaler, f)

