#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb  7 19:27:50 2023

@author: sumeyra
"""

import rospy
from geometry_msgs.msg import Twist
from basit_uygulamalar.srv import CemberHareket

def cemberFonk(istek):
    hiz_mesaji=Twist()
    lineer_hiz=0.5
    hiz_mesaji.linear.x= lineer_hiz
    yaricap = istek.yaricap
    #w = v/r
    hiz_mesaji.angular.z = lineer_hiz / yaricap
    while not rospy.is_shutdown():
        pub.publish(hiz_mesaji)
        

rospy.init_node("cember_hareket")
pub = rospy.Publisher("cmd_vel",Twist,queue_size =10)
rospy.Service("cember_servisi",CemberHareket,cemberFonk)
rospy.spin()
