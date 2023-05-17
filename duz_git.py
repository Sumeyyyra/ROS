#!/usr/bin/env python3
# -- coding: utf-8 --
"""
Created on Sat Feb  4 15:40:41 2023
     Tek eksende hareket I
@author: sumeyra
"""
import rospy
from geometry_msgs.msg import Twist

def hareket():
    rospy.init_node("duz_git") #dugum baslatti
    pub = rospy.Publisher("cmd_vel",Twist,queue_size=10) #hiz mesajları burdan yayinlaniyo
    hiz_mesaji = Twist() # bu tipte mesaj yayinlicaz dedik
    hiz_mesaji.linear.x = 0.5 # x yönünde 0.5 lik hızla giriyo
    mesafe = 20
    yer_degistirme=0
    t0 = rospy.Time.now().to_sec() #güncel zamani saniye olarak aldık
    while(mesafe > yer_degistirme):    
        pub.publish(hiz_mesaji)
        t1 = rospy.Time.now().to_sec()
        yer_degistirme = hiz_mesaji.linear.x * (t1-t0)
    hiz_mesaji.linear.x = 0.0
    pub.publish(hiz_mesaji)
    
    
hareket()
 

    