import numpy as np
import cv2
def warp(img):
    # Define calibraction box in source (original) and distination (desired or warped) coordinates
    img_size = w, h = (img.shape[1], img.shape[0])
    
    src = np.float32([[w, h-10], #bottom right
                      [0,h-10], # 
                      [558, 476],
                      [723, 476]])
    dst = np.float32([[w, h],
                      [0, h],
                      [230,0],
                      [w-200, 0]])

    # Compute the prespective transform, M
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    # Creat warped image - uses linear interpolation
    warped = cv2.warpPerspective(img, M, img_size, flags=cv2.INTER_LINEAR)
    
    return warped

def unwarp(img):
    # Define calibraction box in source (original) and distination (desired or warped) coordinates
    img_size = w, h = (img.shape[1], img.shape[0])
    
    src = np.float32([[w, h-10], #bottom right
                      [0,h-10], # 
                      [558, 476],
                      [723, 476]])
    dst = np.float32([[w, h],
                      [0, h],
                      [230,0],
                      [w-200, 0]])

    Minv = cv2.getPerspectiveTransform(dst, src)
    # Creat warped image - uses linear interpolation
    unwarped = cv2.warpPerspective(img, Minv, img_size, flags=cv2.INTER_LINEAR)
    
    return unwarped
def sliding_window_search(bird_eye, height, rline, lline):
    # Take the histogram of the bottom half of the image
    histogram = np.sum(bird_eye[height/2:,:], axis=0)
    # Find the peaks of the left and right halves of the histogram.
    # These will be the starting point for the left and right lines
    midpoint = np.int(histogram.shape[0]/2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    #print('rightx_base',rightx_base)
    # Choose the number of sliding windows
    nwindows = 9
    # Set height of windows
    window_height = np.int(height/nwindows)
    # Identify the x and y position of all nonzero pixels in the image
    nonzeroy, nonzerox = bird_eye.nonzero()  # importantment setp!
    # Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base
    # Set the width of the windows +/- margin
    margin = 100
    # Set the minimun number of pixels found to recenter window
    minpix = 50
    # Creat empty list to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

    # Step through winsows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y 
        win_y_low = height - (window+1)*window_height
        win_y_high = height - window*window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        # Identify the nonzero pixels in x and y within the window
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & \
                            (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & \
                            (nonzerox < win_xright_high)).nonzero()[0]
        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

    # Concatenate the arrays of indices
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)
    
    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]
    
    detected = True
    if not list(left_lane_inds):
        detected = False
        lline.update(detected, leftx, lefty)
    else:
        lline.update(detected, leftx, lefty)
        
    detected = True
    if not list(right_lane_inds):
        detected = False
        rline.update(detected, rightx, righty)
    else:
        rline.update(detected, rightx, righty)

def previous_fit_window_search(bird_eye, rline, lline):
    margin = 100
    left_coeff = lline.ave_poly_fit
    right_coeff = rline.ave_poly_fit
    
    nonzeroy, nonzerox = bird_eye.nonzero()
    
    left_lane_inds = (
    (nonzerox > (left_coeff[0]*(nonzeroy**2) + left_coeff[1]*nonzeroy + left_coeff[2] - margin)) & 
    (nonzerox < (left_coeff[0]*(nonzeroy**2) + left_coeff[1]*nonzeroy + left_coeff[2] + margin)))
    right_lane_inds = (
    (nonzerox > (right_coeff[0]*(nonzeroy**2) + right_coeff[1]*nonzeroy + right_coeff[2] - margin)) & 
    (nonzerox < (right_coeff[0]*(nonzeroy**2) + right_coeff[1]*nonzeroy + right_coeff[2] + margin)))
    
    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]
    
    detected = True
    if not list(left_lane_inds):
        detected = False
        lline.update(detected, leftx, lefty)
    else:
        lline.update(detected, leftx, lefty)
        
    detected = True
    if not list(right_lane_inds):
        detected = False
        rline.update(detected, rightx, righty)
    else:
        rline.update(detected, rightx, righty)

def calculate_offset(height, width, rline, lline):
    img_midpoint = width* 3.7 /700 / 2 # distance(meter) to the left side
    img_height_meter = height*30/720 # y value in meter at the bottom of the image
    bottom_left = lline.ave_poly_fit_meter[0]*img_height_meter**2 + lline.ave_poly_fit_meter[1]*img_height_meter + lline.ave_poly_fit_meter[2]
    bottom_right = rline.ave_poly_fit_meter[0]*img_height_meter**2 + rline.ave_poly_fit_meter[1]*img_height_meter + rline.ave_poly_fit_meter[2]
    car_location = (bottom_right - bottom_left) / 2 + bottom_left
    offset = car_location - img_midpoint # if <0 left, else right
    return offset

def draw_back(warped, rline, lline):
    h, w = warped.shape
    # Generate x and y values for plotting
    ploty = np.linspace(0, h-1, h)
    left_fitx = lline.ave_poly_fit[0]*ploty**2 + lline.ave_poly_fit[1]*ploty + lline.ave_poly_fit[2]
    right_fitx = rline.ave_poly_fit[0]*ploty**2 + rline.ave_poly_fit[1]*ploty + rline.ave_poly_fit[2]
    
    # Create an image to draw the lines on
    warp_zero = np.zeros_like(warped).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
    color_warp2 = np.dstack((warp_zero, warp_zero, warp_zero))
    
    # Draw the left and right line on color_warp
    rline.draw(color_warp,color=[0,0,255])
    lline.draw(color_warp)
    
    # Recast the x and y points into usable format for cv2.fillPoly()
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))

    # Draw the lane onto the warped blank image
    cv2.fillPoly(color_warp2, np.int_([pts]), (0,255, 0))
    
    color_warp = cv2.addWeighted(color_warp, 1, color_warp2, 0.3, 0.)
    unwarped = unwarp(color_warp)
    return unwarped

