#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Camera", cv_image)
        cv2.waitKey(1)
    except Exception as e:
        print(e)


def camera_display_node():
    rospy.init_node('camera_display_node', anonymous=True)
    rospy.Subscriber('/camera/image_raw', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        camera_display_node()
    except rospy.ROSInterruptException:
        pass