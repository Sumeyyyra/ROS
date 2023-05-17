#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Feb  4 16:22:40 2023
tek eksen 2
@author: sumeyra
"""
import rospy 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry #robotun konumunu alıcak 
from basit_uygulamalar.msg import Mesafe

class HedefeGit():
    def __init__(self):
        rospy.init_node("duz_git")
        self.hedef_konum = 0.0
        self.guncel_konum= 0.0
        self.kontrol = True
        rospy.Subscriber("odom",Odometry,self.odomCallback)
        rospy.Subscriber("mesafe_git",Mesafe,self.mesafeCallback)
        pub = rospy.Publisher("cmd_vel",Twist,queue_size=10)
        hiz_mesaji = Twist()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():# rospy kapatılana kadar yayınlanmasın ısağlıcaz
            if self.kontrol:
                hiz_mesaji.linear.x =0.5
                pub.publish(hiz_mesaji)
            else:
                hiz_mesaji.linear.x=0.0
                pub.publish(hiz_mesaji)
                rospy.loginfo("hedefteyiz")
            rate.sleep() #tanımladığımız döngü hızında gerçekleşmesi çin bunuu yazdık 
 
       #rospy.spin() sürekli dönmesini sağlıyo
        
    def odomCallback(self,mesaj):
        self.guncel_konum = mesaj.pose.pose.position.x
        if self.guncel_konum <= self.hedef_konum:
            self.kontrol = True
        else:
            self.kontrol = False
    
    def mesafeCallback(self,mesaj):
        self.hedef_konum = mesaj.mesafe
        
            
try: 
    HedefeGit()      
except rospy.ROSInterruptException:
    print("dugum sonlandi")