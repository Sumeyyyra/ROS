#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Renk Uzayları ve Uzay Dönüşümleri
"""

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class Kamera():
    def __init__(self):
        rospy.init_node("kamera_dugumu")
        self.bridge = CvBridge()
        rospy.Subscriber("camera/rgb/image_raw",Image,self.kameraCallback)
        rospy.spin()
        
    def kameraCallback(self,mesaj):
        img = self.bridge.imgmsg_to_cv2(mesaj,"bgr8")
        b,g,r = cv2.split(img) #gelen görünyü b g r kanalların ayırdı
        img4 = cv2.merge((b,g,r))#gelen b g r kanallarını birleştirir
#        img2 = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
#        img3 = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        cv2.imshow("BGR",img)
        cv2.imshow("B",b)
        cv2.imshow("G",g)
        cv2.imshow("R",r)
        cv2.imshow("Birlestirilmis",img4)
#        cv2.imshow("Gri",img2)
#        cv2.imshow("HSV",img3)
        cv2.waitKey(1)

Kamera()