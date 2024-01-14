#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

import cv2
import numpy as np

counter = 0
def image_callback(msg):

    global counter

    try:
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg, "bgr8")

        if counter <= 100:
            counter +=1
            rospy.loginfo(f"đọc video từ pulisher: {counter}")

        else:
            (H, W) = image.shape[:2]
            blob = cv2.dnn.blobFromImage(image, 0.007843, (W, H), 127.5)
            detector.setInput(blob)
            person_detections = detector.forward()
            
            for i in range(person_detections.shape[2]):
                confidence = person_detections[0, 0, i, 2]
                if confidence > 0.8:
                    idx = int(person_detections[0, 0, i, 1])
                    if CLASSES[idx] != "person":
                        rospy.logwarn("No person found")
                        cv2.imwrite(f'data/frame_template.jpg', image)
                        continue
                    rospy.logwarn("Found someone")
                    person_box = person_detections[0, 0, i, 3:7] * np.array([W, H, W, H])
                    (startX, startY, endX, endY) = person_box.astype("int")
                    saved_frame = image[startY:endY, startX:endX]
                    
                    cv2.imwrite(f'/home/nhathai/ros_ws/data/frame_template.jpg', saved_frame)

                    my_task()
    except Exception as e:
        print(e)

def my_task():
    # Thực hiện nhiệm vụ của bạn ở đây
    rospy.loginfo("Nhiệm vụ đã hoàn thành!")
    rospy.signal_shutdown("Nhiệm vụ đã hoàn thành. Node sẽ dừng.")

if __name__ == '__main__':
    try:
        prototxt_path = "/home/nhathai/ros_ws/model/MobileNetSSD_deploy.prototxt"
        model_path = "/home/nhathai/ros_ws/model/MobileNetSSD_deploy.caffemodel"
        CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person"]

        detector = cv2.dnn.readNetFromCaffe(prototxt_path, model_path)
        rospy.init_node('detect_person', anonymous=True)
        rospy.Subscriber("camera/color/image_raw", Image, callback=image_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass