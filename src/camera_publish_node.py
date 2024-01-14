#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time



def publish_webcam_image():
    
    rospy.init_node('webcam_publisher_node', anonymous=True)
    pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    rate = rospy.Rate(30)  # Thay đổi tốc độ publish ở đây nếu cần

    cap = cv2.VideoCapture(0)  # Mở webcam, số 0 thường là webcam mặc định

    bridge = CvBridge()
    prev_time = 0
    camera_available = False  # Biến cờ để theo dõi trạng thái của camera

        
    while not rospy.is_shutdown():
        if not camera_available:
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():  # Kiểm tra nếu không thể mở camera
                rospy.logwarn("Không thể mở camera! Vui lòng kiểm tra kết nối.")
                time.sleep(2)  # Chờ 2 giây trước khi kiểm tra lại
                #cap = cv2.VideoCapture(0)  # Mở webcam, số 0 thường là webcam mặc định
                continue
            else:
                camera_available = True
                rospy.loginfo("Camera đã được kết nối!")

        ret, frame = cap.read()  # Đọc frame từ webcam
            
        if ret:
            
            # Tính FPS
            current_time = time.time()
            fps = 1 / (current_time - prev_time) if prev_time else 0
            prev_time = current_time
            #rospy.loginfo(f"FPS: {fps}")

            #frame  = cv2.rotate(frame ,cv2.ROTATE_90_COUNTERCLOCKWISE)

            # Hiển thị FPS trên video
            cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Chuyển đổi frame sang định dạng ROS Image
            ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

            # Publish frame lên topic
            pub.publish(ros_image)

        else:
            camera_available = False
            rospy.logwarn("Không thể mở camera! Vui lòng kiểm tra kết nối.")

        rate.sleep()

    # Khi kết thúc, giải phóng webcam và đóng cửa sổ OpenCV
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        publish_webcam_image()
    except rospy.ROSInterruptException:
        pass
