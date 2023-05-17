#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: sumeyra
"""

import rospy
from basit_uygulamalar.srv import CemberHareket

rospy.wait_for_service("cember_servisi")
try:
    yaricap= float(input("yaricap gir: "))
    servis = rospy.ServiceProxy("cember_servisi",CemberHareket)
    servis(yaricap)#talepte bulundun
except rospy.ServiceException:
    print("emir")