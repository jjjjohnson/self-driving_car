#!/usr/bin/env python
import rospy
from light_classification.tl_classifier import TLClassifier
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class TLTest(object):
    def __init__(self):
    	sub1 = rospy.Subscriber('/image_raw', Image, self.classify)

    	self.bridge = CvBridge()

    	self.light_classifier = TLClassifier()

    	rospy.spin()

    def classify(self, msg):
    	self.camera_image.encoding = "rgb8"
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        self.light_classifier.get_classification(cv_image)

        # rospy.loginfo()