#**Finding Lane Lines on the Road** 
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

<img src="laneLines_thirdPass.jpg" width="480" alt="Combined Image" />

When we drive, we use our eyes to decide where to go.  The lines on the road that show us where the lanes are act as our constant reference for where to steer the vehicle.  Naturally, one of the first things we would like to do in developing a self-driving car is to automatically detect lane lines using an algorithm.

In order to detected left and right lines, various openCV functions has heen deployed: `cv2.Canny`, `cv2.GaussianBlur`, `cv2.fillPoly`, `cv2.line`, `cv2.HoughLinesP` ect.

The overall process of Lane finding is:

 - Convert the RGB image to gray scale.
 - Apply `gaussian_blur` to remove noise.
 - Apply `cv2.Canny` to get gradient image.
 - Only keeps the region of the gradient image defined by the polygon.
 - Detect lines within region of image using `cv2.HoughLinesP` 
 - Calculate the slope of each line and filter line based on its slope. Then subgroup lines in two lists(left lines and right lines), which have predefined length(`deque`)
 - Find the median slope and median x value of left and right line lists, respecively, and draw the line back to the frame image.