#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist, Point
from matplotlib.patches import Rectangle
import math

lidar_data = None
center_x, center_y = 0, 0
threshold_distance = 0.5
human_position = [0, 0, 0]

def lidar_callback(data):
    global lidar_data
    lidar_data = data

def human_position_callback(data):
    global human_position
    human_position = [data.x, data.y,data.z]

def tinh_goc_tap(O, A):
    # Tính độ dài các cạnh của tam giác vuông OA và Ox
    canh_ngang = A[0] - O[0]  # Độ dài theo trục x
    canh_doc = A[1] - O[1]    # Độ dài theo trục y

    # Tính góc tạp bởi sử dụng hàm atan2
    goc_tap = math.atan2(canh_doc, canh_ngang)
    
    # Chuyển đổi radian sang độ
    goc_tap_deg = math.degrees(goc_tap)

    return goc_tap_deg

def cung_dau(a, b):
    if (a * b) >= 0:
        return 1
    else :
        return 0


def plot_lidar_data():
    plt.clf()
    ranges = np.array(lidar_data.ranges)
    angle_min = lidar_data.angle_min
    angle_max = lidar_data.angle_max
    angle_increment = lidar_data.angle_increment
    angles = np.linspace(angle_min, angle_max, len(ranges))

    xs = ranges * np.cos(angles)
    ys = ranges * np.sin(angles)

    xs -= center_x
    ys -= center_y

    ranges = np.nan_to_num(ranges, nan=0.0, posinf=0.0, neginf=0.0)

    distances_from_center = np.sqrt(xs**2 + ys**2)
    close_to_center = distances_from_center < threshold_distance

    

    plt.scatter(xs[~close_to_center], ys[~close_to_center], s=2, c='blue', alpha=0.5)
    plt.scatter(0, 0, s=50, c='red', marker='x', label='Center')
    plt.scatter(human_position[0], human_position[1], s=100, c='green', marker='o', label='Human')
    plt.plot([-3, 3], [0, 0], linestyle='--', color='gray')
    plt.plot([0, 0], [-3, 3], linestyle='--', color='gray')

    # Add a square
    height,width = 0.6,0.4 # Define the side length of the square
    square = Rectangle((-height/2, -width /2), height,width, edgecolor='orange', facecolor='none')
    plt.gca().add_patch(square)

    # Add a square
    height,width = 2,2 # Define the side length of the square
    square = Rectangle((-height/2, -width /2), height,width, edgecolor='orange', facecolor='none')
    plt.gca().add_patch(square)

    # Identify points within the square
    
    points_inside_square = np.logical_and(np.abs(xs) < height / 2, np.abs(ys) < width / 2)

    # Lọc các điểm có xs > 0
    filtered_points = xs[points_inside_square] > 0

    # Tạo mảng chứa các điểm xs > 0
    xs_positive = xs[points_inside_square][filtered_points]
    ys_positive=ys[points_inside_square][filtered_points]

    # Plot các điểm trong hình vuông
    plt.scatter(xs[points_inside_square], ys[points_inside_square], s=5, c='green', alpha=0.8)

    # Plot các điểm có xs > 0
    plt.scatter(xs_positive, ys_positive, s=5, c='black', alpha=0.8)

    

    for i in range(len(xs_positive)):
        if ys_positive[i]:
            angular = tinh_goc_tap((center_x,center_y),(xs_positive[i], ys_positive[i]))#y<0 --> angular <0 --> xoay phải
            print(f"Góc của điểm {xs_positive[i], ys_positive[i]} là: {angular} độ")

            if cung_dau(angular,human_position[2]):
                print("nguuu")
                abs_angular = abs(angular) 
                abs_human_position = abs(human_position[2])
                if abs_angular> abs_human_position:
                    distance_obs = np.sqrt(xs_positive[i]**2+ ys_positive[i]**2)
                    if distance_obs < 1:
                        if ys_positive[i] >0:
                            twist_msg.angular.z = -(-0-angular)
                        if ys_positive[i]<0:
                             twist_msg.angular.z = -0-angular
                    else :
                        twist_msg.angular.z = human_position[2]
                else :
                    print("Thien")
                    twist_msg.angular.z = human_position[2] - (90-angular)
            else:
                twist_msg.angular.z = human_position[2]

            lower_limit = -90
            upper_limit = 90
            limited_angular_z = max(min(twist_msg.angular.z, upper_limit), lower_limit)

            twist_msg.angular.z = limited_angular_z
            twist_msg.linear.x=2000
            print(twist_msg)
            #distances_from_center tìm điểm gần ox nhất

            


    #plt.scatter(xs[close_to_center], ys[close_to_center], s=5, c='red', alpha=0.8)  



    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('LiDAR Scan Data with Objects Close to Center Highlighted in Red')
    plt.axis('equal')
    plt.legend()
    plt.pause(0.00001)
    plt.draw()

def control_robot():
    rospy.init_node('lidar_listener', anonymous=True)
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, lidar_callback)
    rospy.Subscriber("/human_position_topic", Point, human_position_callback)
    
    plt.ion()
    plt.show()

    while not rospy.is_shutdown():
        if lidar_data is not None:
            plot_lidar_data()
            pub.publish(twist_msg)

            

if __name__ == '__main__':
    twist_msg = Twist()
    control_robot()
