import numpy as np
import cv2

def abs_sobel_thresh(gray, orient='x', sobel_kernel=3, thresh=(0, 255)):
    """Implement sobel gradient along x or y dimension."""
    #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if orient == 'x':
        sobel = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    else:
        sobel = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    abs_sobel = np.absolute(sobel)
    scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))
    grad_binary = np.zeros_like(scaled_sobel)
    grad_binary[(scaled_sobel >= thresh[0]) & (scaled_sobel <= thresh[1])] = 1
    return grad_binary

def mag_thresh(gray, sobel_kernel=3, mag_thresh=(0, 255)):
    """Implement sobel gradient along magnitude."""
    #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    mag_sobel = np.sqrt(sobelx**2 + sobely**2)
    scaled_sobel = np.uint8(255*mag_sobel/np.max(mag_sobel))
    mag_binary = np.zeros_like(scaled_sobel)
    mag_binary[(scaled_sobel >= mag_thresh[0]) & (scaled_sobel <= mag_thresh[1])] = 1
    return mag_binary

def dir_threshold(gray, sobel_kernel=3, thresh=(0, np.pi/2)):
    """Implement threshold on direction arctan2."""
    #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    abs_sobelx = np.absolute(sobelx)
    abs_sobely = np.absolute(sobely)
    dgrad = np.arctan2(abs_sobely, abs_sobelx)
    dir_binary = np.zeros_like(dgrad)
    dir_binary[(dgrad >= thresh[0]) & (dgrad <= thresh[1])] = 1
    return dir_binary

def saturaction_threshold(image, thresh=(170, 255)):
    hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
    s_channel = hls[:,:,2]
    s_binary = np.zeros_like(s_channel)
    s_binary[(s_channel >= thresh[0]) & (s_channel <= thresh[1])] = 1
    return s_binary

def combined_threshold(image):
    gray_img = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    gradx = abs_sobel_thresh(gray_img, orient='x', sobel_kernel=9, thresh=(40, 100))
    grady = abs_sobel_thresh(gray_img, orient='y', sobel_kernel=9, thresh=(40, 100))
    dir_s = dir_threshold(gray_img, sobel_kernel=19, thresh=(0.9, 1.2))
    s_h = saturaction_threshold(image, thresh=(180, 255))
    
    combined = np.zeros_like(dir_s)
    combined[((gradx == 1) | (grady == 1))  | (s_h == 1) & (dir_s == 1)] = 1
    
    return combined

