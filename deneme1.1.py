#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 10 00:09:52 2023

@author: sumeyra
"""
import rospy 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class LazerVerisi():
    def __init__(self):
        rospy.init_node("lazer_dugumu")
        self.pub =rospy.Publisher("cmd_vel",Twist,queue_size=10)
        self.hiz_mesaji=Twist()
        rospy.Subscriber("scan",LaserScan,self.lazerCallback)
        rospy.spin()
        
    def lazerCallback(self,mesaj):
        sol_on = list(mesaj.ranges[0:9])
        sag_on = list(mesaj.ranges[350:359])
        on= sol_on + sag_on
        sol=list(mesaj.ranges[80:100])
        sag=list(mesaj.ranges[260:280])
        arka=list(mesaj.ranges[170:190])
        min_on =min(on)
        min_sol=min(sol)
        min_sag=min(sag)
        min_arka=min(arka)
        print(min_on,min_sol,min_sag,min_arka)
        if min_on<1.0:
           
            self.hiz_mesaji.linear.x=0.0
            self.pub.publish(self.hiz_mesaji)
        else:
            self.hiz_mesaji.linear.x=0.25
            self.pub.publish(self.hiz_mesaji)
            
        

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
            print(sapma)
            self.hiz_mesaji.linear.x = 0.2
            self.hiz_mesaji.angular.z = -sapma/100
            self.pub.publish(self.hiz_mesaji)
        else:
            self.hiz_mesaji.linear.x = 0.0
            self.hiz_mesaji.angular.z = 0.0
            self.pub.publish(self.hiz_mesaji)
        cv2.imshow("Orjinal",img)
        cv2.imshow("Maske",maske)
        cv2.imshow("Sonuc",sonuc)
        cv2.waitKey(1)
        
        
SeritTakip()        
nesne =LazerVerisi()
