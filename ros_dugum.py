#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS Düğümü Oluşturma
"""

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class Kamera():
    def __init__(self):
        rospy.init_node("kamera_dugumu")#ilk yapman gerekne düğümü başşlatmak
        self.bridge = CvBridge() #köprü kurduk
        rospy.Subscriber("camera/rgb/image_raw",Image,self.kameraCallback)# abone olduk ve image mesajı üzer,nden işlem gerçekleştiren ve abone olunca kamercallback fonskiyonuna giden bir abone tanımladık 
        rospy.spin()#süreklilik sağladık
        
    def kameraCallback(self,mesaj):
        img = self.bridge.imgmsg_to_cv2(mesaj,"bgr8")# bridge ile gelen mesajı bgr8 formatında cv2 ya çevir
        cv2.imshow("Robot Kamerasi",img)
        cv2.waitKey(1)

Kamera()