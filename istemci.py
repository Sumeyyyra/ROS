#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 31 23:16:39 2023

@author: sumeyra
"""

import rospy
from ogretici_paket.srv import GecenZaman

def istekteBulun(x):
    rospy.wait_for_service("zaman")
    try:
        servis = rospy.ServiceProxy("zaman",GecenZaman)
        cevap = servis(x)
        return cevap.gecen_sure
    except rospy.ServiceException:
        print("servisle alakli hata !!!")

hedef = float(input("hedef girin"))
t =istekteBulun(hedef)
print("sure ",t)