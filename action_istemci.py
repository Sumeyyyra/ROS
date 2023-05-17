#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Action Service-Client Uygulaması: Client Düğümü
"""

import rospy
import actionlib
from ogretici_paket.msg import GorevdurumAction, GorevdurumGoal

def bildirimFonksiyonu(bilgi):
    print("Gorev tamamlanma durumu: ", bilgi.oran)

def istekteBulun():
    rospy.init_node("action_istemci_dugumu")
    İstemci = actionlib.SimpleActionClient("gorev",GorevdurumAction)
    İstemci.wait_for_server()
    istek = GorevdurumGoal()
    istek.birim = 10
    İstemci.send_goal(istek,feedback_cb=bildirimFonksiyonu)
    İstemci.wait_for_result()
    x = İstemci.get_result().sonuc
    return x

cikti = istekteBulun()
print("Gorevin son durumu: ", cikti)
