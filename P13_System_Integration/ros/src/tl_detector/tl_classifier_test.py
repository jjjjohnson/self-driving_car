#!/usr/bin/env python
import rospy
from light_classification.tl_classifier import TLClassifier
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import threading
import cPickle as pickle

class TLTest(object):
    def __init__(self):
        rospy.init_node('tl_classifier_test')

    	sub1 = rospy.Subscriber('/image_raw', Image, self.classify)
        # sub2 = rospy.Subscriber('/current_pose', PoseStamped, self.save_images)

    	self.bridge = CvBridge()
        self.camera_image = None

        self.light_classifier_is_ready = False

        def wait_for_loading_network():
            self.light_classifier_is_ready = True
            print("TL classifier loaded")
            t.cancel()

        print("Loading TL classifier (5 second timer)")
        t = threading.Timer(5.0, wait_for_loading_network)
        t.daemon = True
        t.start()

    	self.light_classifier = TLClassifier()
        # self.images = []

    	rospy.spin()

    def classify(self, msg):
        self.camera_image = msg
        if self.light_classifier_is_ready:
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
            # self.images.append(cv_image)
            self.light_classifier.class_visualize(cv_image)
            cv2.imshow('Bag images', cv2.resize(cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR), (800, 600)))
            # self.light_classifier.get_classification(cv_image)
            cv2.waitKey(1)
                

        # rospy.loginfo()

    # def save_images(self, msg):
    #     """Save the images at the end of the sequence"""
    #     if msg.header.seq == 5085:
    #         pickle.dump(self.images, open("save.p", "wb"))
    #         print("Images saved")

if __name__ == '__main__':
    try:
        TLTest()

    except rospy.ROSInterruptException:
        rospy.logerr('Could not start tl_classifier_test node.')