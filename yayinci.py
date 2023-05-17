#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 31 19:45:27 2023

@author: sumeyra
"""


import rospy
from ogretici_paket.msg import BataryaDurum


def mesajYayinla():
    rospy.init_node("yayinci_dugumu",anonymous=True)
    pub = rospy.Publisher("batarya_konusu",BataryaDurum,queue_size=10)
    rate =rospy.Rate(1)
    while not rospy.is_shutdown():
        durum ="%25"
        rospy.loginfo(durum)
        pub.publish(durum)
        rate.sleep()

mesajYayinla()
