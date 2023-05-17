#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr  7 21:47:57 2022
@author: kutay
"""

import cv2 
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

#sari = np.uint8([[[0,255,255]]])
#hsv = cv2.cvtColor(sari,cv2.COLOR_BGR2HSV
class SeritTakip():
    def __init__(self):
        rospy.init_node("serit_takip")
        self.bridge = CvBridge()
        rospy.Subscriber("camera/rgb/image_raw",Image,self.kameraCallback)
        self.pub = rospy.Publisher("cmd_vel",Twist,queue_size = 10)
        self.hiz_mesaji = Twist()
        rospy.spin()
    def kameraCallback(self,mesaj):
        img = self.bridge.imgmsg_to_cv2(mesaj,"bgr8")
        k1 = np.float32([[5,200],[605,200],[5,400],[605,400]])
        k2 = np.float32([[0,0],[640,0],[0,480],[640,480]])
        M = cv2.getPerspectiveTransform(k1,k2)
        img = cv2.warpPerspective(img,M,(640,480))
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        alt_sari = np.array([20,100,100])
        ust_sari = np.array([40,255,255])
        maske = cv2.inRange(hsv,alt_sari,ust_sari)
        sonuc = cv2.bitwise_and(img,img,mask=maske)
        h,w,d = img.shape
        cv2.circle(img,(int(w/2),int(h/2)),5,(0,0,255),-1)
        M = cv2.moments(maske)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(img,(cx,cy),5,(255,0,0),-1)
            sapma = cx - w/2
            self.hiz_mesaji.linear.x = 0.2
            self.hiz_mesaji.angular.z = -sapma/100
            self.pub.publish(self.hiz_mesaji)
        else:
            self.hiz_mesaji.linear.x = 0.0
            self.hiz_mesaji.angular.z = 0.0
            self.pub.publish(self.hiz_mesaji)
        cv2.imshow("orijinal",img)
        cv2.imshow("makse",maske)
        cv2.imshow("sonuc",sonuc)
        cv2.imshow("pers",img)
        cv2.waitKey(1)
        
        
SeritTakip()