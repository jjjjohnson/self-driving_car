#**Behavioral Cloning** 


---

**Behavioral Cloning Project**

The goals / steps of this project are the following:
* Use the simulator to collect data of good driving behavior
* Build, a convolution neural network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./output/nvidia.png =700x "Nvidia architecture"
[image2]: ./output/center_driving.png
[image3]: ./output/recover.png
[image4]: ./output/fliped.png
[image5]: ./output/distribution.png =700x
[image6]: ./output/everned_dis.png =700x

###Model Architecture and Training Strategy

####1. An appropriate model architecture has been employed

My model consists of a convolution neural network with 3x3 and 5x5 filter sizes and depths between 24 and 64 (model.py lines 62-74) 

The model includes RELU layers to introduce nonlinearity (code line 63-84), and the data is normalized in the model using a Keras lambda layer (code line 64). 

####2. Attempts to reduce overfitting in the model

Almost all the connections between layers have dropout with keep_probability 50% and 20%.

The model was trained and validated on different data sets to ensure that the model was not overfitting (code line 19). The model was tested by running it through the simulator and ensuring that the vehicle could stay on the track.

####3. Model parameter tuning

The model used an adam optimizer, so the learning rate was not tuned manually (model.py line 87).

####4. Appropriate training data

Training data was chosen to keep the vehicle driving on the road. I used a combination of center lane driving, recovering from the left and right sides of the road.

One thing worth noticing in the training data is the very inbalance of the steering angle. Of the total 8036 collections, 4361 of which is 0, which will mislead the model the predict every future collection to be 0.

![image5]

So I introduce a bias term. If `abs(steering_angle) + bias < np.random.rand()`, skip this data point. In this way we get a more evenly distributed steering angle at the cost of introduce some duplicated bigger steering angle.

![image6]
###Model Architecture and Training Strategy

####1. Solution Design Approach

The overall strategy for deriving a model architecture was to:

- 1. Help model generalize better against overfitting.
- 2. Provide enough data so that the model has a chance to learn the right way to drive.
- 3. Ensure the model is deep and complicated enough to be able to learn.

My first step was to use a convolution neural network model similar to the NVIDIA architecture. I thought this model might be appropriate because NVIDIA has used it to drive in the real car!

In order to gauge how well the model was working, I split my image and steering angle data into a training and validation set. I found that my first model had a low mean squared error on the training set but a high mean squared error on the validation set. This implied that the model was overfitting. 

To combat the overfitting, I insert dropout layer to every connections between layers.

Then I stop the training when the training loss and validation loss is almost the same.


The final step was to run the simulator to see how well the car was driving around track one. There were a few spots where the vehicle fell off the track:

- 1. When the car went through bridge, where the road texture is distinctly different from other sections of track.
- 2. When the car went through the transition between curve and straight road.

To improve the driving behavior in these cases, I add the training data of this part by recording driving thourough multiple times.

At the end of the process, the vehicle is able to drive autonomously around the track without leaving the road.

####2. Final Model Architecture

The final model architecture (model.py lines 18-24) consisted of a convolution neural network with the following 5 convolutional layers as showen in the image below

Here is a visualization of the architecture (note: visualizing the architecture is optional according to the project rubric)

![alt text][image1]

####3. Creation of the Training Set & Training Process

To capture good driving behavior, I first recorded two laps on track one using center lane driving. Here is an example image of center lane driving:

![alt text][image2]

I then recorded the vehicle recovering from the left side and right sides of the road back to center so that the vehicle would learn to .... These images show what a recovery looks like starting from ... :

![alt text][image3]


Then I repeated this process on track two in order to get more data points.

To augment the data sat, I also flipped images and angles thinking that this would ... For example, here is an image that has then been flipped:

![alt text][image4]


After the collection process, I had 8036 number of data points. I then preprocessed this data bynormalization  using keras lambda layer(model.py lines 64) and cropping out top 70 rows and bottom 25 rows using Keras Cropping2D layer. This is really efficient because it is done parallely using the GPU.

Under the assumption that a small variance is OK in case of steering angle (no absolute right answer). I introduced `angle += np.random.normal(loc=0, scale=0.3)` to help the model generalize better.

I finally randomly shuffled the data set and put 20% of the data into a validation set. 

I used this training data for training the model. The validation set helped determine if the model was over or under fitting. The ideal number of epochs was 10 as evidenced by the fact that after 10 the validaction did not decrease any more. I used an adam optimizer so that manually training the learning rate wasn't necessary.
