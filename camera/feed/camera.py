#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("Drone Camera Feed", cv_image)
        cv2.waitKey(1)
    except Exception as e:
        rospy.logerr("Error converting image: %s", e)

if __name__ == "__main__":
    rospy.init_node('simple_image_viewer', anonymous=True)
    rospy.Subscriber("/plutocamera/image_raw", Image, image_callback)
    rospy.loginfo("Subscribed to /plutocamera/image_raw")
    rospy.spin()
    cv2.destroyAllWindows()
