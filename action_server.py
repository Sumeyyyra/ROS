#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb  1 15:44:16 2023

@author: sumeyra
"""

import rospy
import actionlib 
from ogretici_paket.msg import GorevdurumAction,GorevdurumFeedback,GorevdurumResult

class ActionServer():
    def __init__(self):
        rospy.init_node("action_server_dugumu")
        self.a_server = actionlib.SimpleActionServer("gorev",GorevdurumAction,auto_start=False,execute_cb=self.cevapUret)
        self.a_server.start()
        rospy.spin()
        
    def cevapUret(self,istek):
        geri_bildirim= GorevdurumFeedback()
        sonuc = GorevdurumResult()
        rate = rospy.Rate(1)
        for i in range(1,istek.birim):
            durum = "%"+ str(i*100/istek.birim)
            geri_bildirim.oran = durum
            self.a_server.publish_feedback(geri_bildirim)
            rate.sleep()
        sonuc.sonuc = "gorev tammalandi."
        self.a_server.set_succeeded(sonuc)
        
a_s = ActionServer()