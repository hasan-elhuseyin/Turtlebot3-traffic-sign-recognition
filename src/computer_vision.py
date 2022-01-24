#!/usr/bin/env python3

# Ros libraries
from matplotlib.pyplot import imshow
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
# Computer Vision libraries
from keras.models import load_model
from PIL import Image as PILImage, ImageOps
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError


class ComputerVision:

    def __init__(self):
        # Load the model
        self.modelPath = "/home/hasan/catkin_ws/src/turtlebot3-traffic-sign-recognition/keras_model/keras_model.h5"
        self.model = load_model(self.modelPath)

        # Create the array of the right shape to feed into the keras model
        self.data = np.ndarray(shape=(1, 224, 224, 3), dtype=np.float32)
        self.imageSub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback_computer_vision)
        self.pub = rospy.Publisher("/classification_result", String, queue_size=10)
        self.rate = rospy.Rate(10)
        print("computer vision node initialized")

    def callback_computer_vision(self, msg):
        self.bridge = CvBridge()
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        #resizing the image to be at least 224x224 and then cropping from the center
        self.image = cv2.resize(self.cv_image, (224, 224))
        
        #self.size = (224, 224)
        #self.image = ImageOps.fit(self.cv_image, self.size, PILImage.ANTIALIAS)
        #turn the image into a numpy array
        self.image_array = np.asarray(self.image)
        # Normalize the image
        self.normalized_image_array = (self.image_array.astype(np.float32) / 127.0) - 1
        # Load the image into the array
        self.data[0] = self.normalized_image_array

        # run the inference
        self.prediction = self.model.predict(self.data)

        # check the result and publish it
        if self.prediction[0][0] >= self.prediction[0][1]:
            #print("left sign " + str(self.prediction[0][0]))
            self.pub.publish("left")
        else:
            #print("right sign " + str(self.prediction[0][1]))
            self.pub.publish("right")


#********************************************************************************************
if __name__ == '__main__':
    rospy.init_node("computer_vision_node", anonymous=True)

    x = ComputerVision()

    rospy.spin()

