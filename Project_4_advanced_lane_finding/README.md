## Project 4 Advanced Lane Finding

---


The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./output_images/undistorted.png =700x "Undistorted"
[image2]: ./output_images/test1.png "Road Transformed"
[image3]: ./output_images/binary_combo.png "Binary Example"
[image4]: ./output_images/wraped_img.png =700x "Warp Example"
[image5]: ./output_images/slide_search.png "Fit Visual"
[image6]: ./output_images/color_fit_lines.png "Out put"
[video1]: ./project_video_output.mp4 "Video"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) 

---

###Camera Calibration

The code for this step is contained in lines #5 through #34 of the file called [calibrate_utility](calibrate_utility.py).  

I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.  

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result: 

![image1]

###Pipeline (single images)

####1. Provide an example of a distortion-corrected image.
To demonstrate this step, I will describe how I apply the distortion correction to one of the test images like this one:
![alt text][image2]
####2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.
I used a combination of color and gradient thresholds to generate a binary image (thresholding steps at lines #47 through #57 of a file called [binary_threshold](binary_threshold.py)).  Here's an example of my output for this step. 

![alt text][image3]

####3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The code for my perspective transform includes a function called `warp()`, which appears in lines 3 through 20 in the file called [utility](utility.py).  The `warp()` function takes as inputs an image (`img`).  I chose the hardcode the source and destination points in the following manner in side the warp() function:

```
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

```
This resulted in the following source and destination points:


| Source        | Destination   | 
|:-------------:|:-------------:| 
| 1280, 710     | 1280, 720     | 
| 0, 710        | 0, 720        |
| 558, 476      | 230, 0        |
| 723, 476      | 1080, 0       |



I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

![alt text][image4]

####4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

I use two functions to identify the lane-line pixels: `sliding_window_search()` and `previous_fit_window_search()` in the file [utility](utility.py)(line #42 through #113 and #115 through #147, respectively). `sliding_window_search()` is computationally expensive, so I call it only at the beginning of the vodeo or it cannot find lane-pixel in either side of the rode or the calculated number seems wrong for 5 times in a row. Otherwise, I use the `previous_fit_window_search()` by leveraging the fitted parameters to implement a partial mask.
I've incorported polymeral fit in the class [Line](Line.py) line #41 through #57 and set it to be the property of the class so that we can call it directly.

![alt text][image5]

####5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

I did this in lines #55 through #57 in my code in [Line](Line.py)

####6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

I implemented this step in lines #158 through #184 in my code in [utility](utility.py) in the function `dwaw_back()`.  Here is an example of my result on a test image:

![alt text][image6]

---

###Pipeline (video)

####1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Here's a [link to my video result](./project_video_output.mp4)

---

###Discussion

####1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

The most important challenge I encountered is to distinguish whether the detected lane pixels are corrected. Here, I check the distance between two lanes. It should roughly between 4m and no bigger than 5m. If the calculated frame has abnormal, I disreguard the collected lane pixels and count the number of bad frames. If the number of bad frames exceed 5, it will implement the `sliding_window_search()`, which will try to find lane pixels from scratch.

