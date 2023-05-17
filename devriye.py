#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb  7 20:22:42 2023

@author: sumeyra
"""
import rospy
from geometry_msgs.msg import Twist

def volta():
    rospy.init_node("volta")
    pub = rospy.Publisher("cmd_vel",Twist,queue_size= 10)
    hiz_mesaji =Twist()
    robot_hiz=0.30
    volta_uzunluk= rospy.get_param("/VoltaUzunluk")#voltauzunluğa arametredeki degeri atadık
    volta_sayisi= rospy.get_param("/VoltaSayisi")
    sayac =0
    rospy.loginfo("devriye basladi")
    
    while sayac < volta_sayisi:
        t0 = rospy.Time.now().to_sec()
        yer_degistirme =0 
        if sayac %2 ==0:
            hiz_mesaji.linear.x=robot_hiz
        else:
            hiz_mesaji.linear.x=-robot_hiz
        while yer_degistirme < volta_uzunluk:
            pub.publish(hiz_mesaji)
            t1 = rospy.Time.now().to_sec()  
            yer_degistirme= robot_hiz * (t1-t0)
        hiz_mesaji.linear.x=0
        pub.publish(hiz_mesaji)
        sayac = sayac+1
    
    rospy.loginfo("devriye bitti")
    rospy.is_shutdown()
    
volta()
        
            
