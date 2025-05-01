#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()
hand_cascade = cv2.CascadeClassifier('src/palm.xml')

def image_callback(msg):
    try:
        # Convert ROS image to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Resize to speed up processing
        small_frame = cv2.resize(cv_image, (320, 240))

        # Convert to grayscale
        gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)

        # Detect hands
        hands = hand_cascade.detectMultiScale(gray, 1.1, 3)

        # Draw rectangles
        for (x, y, w, h) in hands:
            cv2.rectangle(small_frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # Display the result
        cv2.imshow("Hand Detection (Optimized)", small_frame)
        cv2.waitKey(1)

    except Exception as e:
        rospy.logerr("Error: %s", e)

if __name__ == "__main__":
    rospy.init_node('hand_detector_optimized', anonymous=True)
    rospy.Subscriber("/plutocamera/image_raw", Image, image_callback)
    rospy.loginfo("Hand detection node started...")
    rospy.spin()
    cv2.destroyAllWindows()
