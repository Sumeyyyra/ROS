#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 31 23:08:52 2023

@author: sumeyra
"""

import rospy
from ogretici_paket.srv import GecenZaman

def GecenZamanFonksiyonu(istek):
    robot_hiz=0.5
    sure = istek.hedef_konum / robot_hiz
    return sure
    
def cevapGonder():
    rospy.init_node("server_dugumu")
    rospy.Service("zaman",GecenZaman,GecenZamanFonksiyonu)
    rospy.spin()
    
cevapGonder()