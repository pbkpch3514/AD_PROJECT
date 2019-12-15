# -*-coding:utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class LineDetector :

    def __init__(self, topic) :
        self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)    
        self.bridge = CvBridge()
        rospy.Subscriber(topic, Image, self.conv_image)
        
        self.roi = np.array([[(0, 315), (639, 305), (170, 260), (460, 260)]],
                    dtype='float32')
        self.birdview_zone = np.array([[(0, 100), (110, 100), (0, -110), (110, -110)]], dtype='float32')
        self.theta = 0
        self.screen = None

    def conv_image(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
    
    def detect_lines(self):
        img = self.cam_img
        
        birdview_size = (100, 210)
        birdview_width, birdview_height = birdview_size
        warp_bird = cv2.getPerspectiveTransform(self.roi, self.birdview_zone)
        birdview = cv2.warpPerspective(img, warp_bird, birdview_size)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (7, 7), 0)
        edges = cv2.Canny(blur, 50, 150, apertureSize=3)
        birdview_edges = cv2.warpPerspective(edges, warp_bird, birdview_size)
        scr_bird = cv2.cvtColor(birdview_edges, cv2.COLOR_GRAY2BGR)
        scr_orgin = edges
        self.screen = scr_orgin
        left, right = -1, -1
        count = 0
        counts = [0 for i in range(50)]
        values = [0.0 for j in range(len(counts))]
        thetas = []
        pointCounts = [0 for k in range(10)]
        pointValues = [0.0 for l in range(len(pointCounts))]
        
        # hough transform
        lines = cv2.HoughLines(birdview_edges, 1, np.pi / 180, 80)
        if lines is not None:
            for line in lines:
                for rho, theta in line:
                    index = int(theta / np.pi * len(counts))
                    counts[index] += 1
                    values[index] += theta
                    count += 1

                    a, b = np.cos(theta), np.sin(theta)
                    x0, y0 = a * rho, b * rho
                    t = (199 - y0) / a
                    x = x0 - t * b

                    index = min(max(int(x / birdview_width * len(pointCounts)), 0), len(pointCounts) - 1)
                    pointCounts[index] += 1
                    pointValues[index] += x
            if count > 0:
                for i in range(2):
                    index = counts.index(max(counts))
                    if counts[index] == 0:
                        break
                    theta = values[index] / counts[index]
                    if theta > np.pi / 2 :
                        theta -= np.pi
                    thetas.append(theta)
                    counts[index] = 0
                theta = sum(thetas) / len(thetas)
            
                self.theta = theta

                points = []
                half = len(pointCounts) // 2

                for cnts, vals in [(pointValues[:half], pointValues[:half]), 
            (pointCounts[half:], pointValues[half:])]:
                    index = cnts.index(max(cnts))
                    point = vals[index] / cnts[index] if cnts[index] > 0 else -1
                    points.append(point)

            left, right = points
        else:
            self.theta *= 0.8

        return self.theta * (float(180) / np.pi), left, right

    def show_images(self, left, right):
        # Display images for debugging purposes;

        if cv2.waitKey(1) & 0xFF == 27:
            exit()
