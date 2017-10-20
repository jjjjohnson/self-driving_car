import numpy as np
import cv2
import glob

def calibrate_cramera():
	"""Find the corners in the images and return the mtx, dist"""

	# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
	objp = np.zeros((6*9,3), np.float32)
	objp[:,:2] = np.mgrid[0:9, 0:6].T.reshape(-1,2)

	# Arrays to store object points and image points from all the images.
	objpoints = [] # 3d points in real world space
	imgpoints = [] # 2d points in image plane.

	# Make a list of calibration images
	images = glob.glob('../camera_cal/calibration*.jpg')

	# Step through the list and search for chessboard corners
	for idx, fname in enumerate(images):
	    img = cv2.imread(fname)
	    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

	    # Find the chessboard corners
	    ret, corners = cv2.findChessboardCorners(gray, (9,6), None)

	    # If found, add object points, image points
	    if ret == True:
	        objpoints.append(objp)
        	imgpoints.append(corners)

    # Do camera calibration given object points and image points
	ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
	return mtx, dist

def undistort(img, mtx, dist):
	"""
	Undistort a img given a camera matrix and distortion coefficient
	:param img: input image
	:param mtx: camera matrix
	:param dist: distortion coefficient
	:return: Undistorted image
	"""
	img_undistorted = cv2.undistort(img, mtx, dist, None, mtx)
	return img_undistorted
