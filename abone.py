#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 31 19:53:26 2023

@author: sumeyra
"""

import rospy
from ogretici_paket.msg import BataryaDurum

def bataryaFonksiyonu(mesaj):
    rospy.loginfo("Robot sarji :%s"%mesaj.batarya)
    

def mesajDinle():
    rospy.init_node("abone_dugumu")
    rospy.Subscriber("batarya_konusu",BataryaDurum,bataryaFonksiyonu)
    rospy.spin()

mesajDinle()
    
    
