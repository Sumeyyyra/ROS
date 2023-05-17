#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr  8 02:20:07 2023

@author: sumeyra
"""
import rospy
import smach
import smach_ros
import cv2 
import math
from std_msgs.msg import String
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
engel_goruldu = False
inverse =False
cizgi_kontrol=0



# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("cmd_vel",Twist,queue_size = 10)
        rospy.Subscriber("camera/rgb/image_raw",Image,self.camCallback)
        rospy.Subscriber("scan",LaserScan,self.engel)
        self.hiz_mesaji = Twist()
        
    def camCallback(self,userdata):
        global cizgi_kontrol
        img = self.bridge.imgmsg_to_cv2(userdata,"bgr8")
        k1 = np.float32([[5,200],[605,200],[5,400],[605,400]])
        k2 = np.float32([[0,0],[640,0],[0,480],[640,480]])
        M = cv2.getPerspectiveTransform(k1,k2)
        img = cv2.warpPerspective(img,M,(640,480))
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        alt_sari = np.array([20,100,100])
        ust_sari = np.array([40,255,255])
        maske = cv2.inRange(hsv,alt_sari,ust_sari)
        h,w,d = img.shape
        cv2.circle(img,(int(w/2),int(h/2)),5,(0,0,255),-1)
        M = cv2.moments(maske)
        
        
        
        if M['m00'] > 0:
            cizgi_kontrol = 0
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(img,(cx,cy),5,(255,0,0),-1)
            sapma = cx - w/2
            self.hiz_mesaji.linear.x = 0.5
            self.hiz_mesaji.angular.z = -sapma/600
            self.pub.publish(self.hiz_mesaji)
        else: 
            cizgi_kontrol = 1
            self.hiz_mesaji.linear.x = 0.0
            self.hiz_mesaji.angular.z = 0.0
            self.pub.publish(self.hiz_mesaji)
           
        
    def execute(self, userdata):
        global engel_goruldu
        global cizgi_kontrol
        rospy.loginfo('Executing state FOO')
        if engel_goruldu:
            self.hiz_mesaji.linear.x = 0.0
            self.hiz_mesaji.angular.z=0.0
            self.pub.publish(self.hiz_mesaji)
            return 'outcome1'
        else :
            if cizgi_kontrol :
                return 'outcome2' 
           
   
    
    
    def engel(self,userdata):
        global engel_goruldu 
        sol_on = list(userdata.ranges[0:9])
        sag_on = list(userdata.ranges[350:359])
        on= sol_on + sag_on
        min_on =min(on)      
        if min_on < 1: 
            engel_goruldu=True 
        else:
            engel_goruldu=False
            
       
# define state Bar
class Bar(smach.State):
    def __init__(self):
        rospy.loginfo('Executing state BAR')
        smach.State.__init__(self, outcomes=['outcome1'])
        rospy.Subscriber("/odom",Odometry,self.odomcb)
        self.pub1 = rospy.Publisher("cmd_vel",Twist,queue_size = 10)
        self.hiz_mesaji = Twist()
        self.nav_message = Int8()
        self.nav_msg_test = String()
        self.pub = rospy.Publisher("/navigasyon",Int8,queue_size=10)
        
    def execute(self,userdata):
       global inverse
       rospy.loginfo('Executing state BAR')
       return 'outcome1'
        
  
        
    def odomcb(self,userdata):
      global inverse
      orientation_q = userdata.pose.pose.orientation
      orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
      (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
      yaw = yaw*(180/math.pi)
      if yaw < 0:
        yaw = yaw + 360
      if (yaw<45 and yaw >0) or (yaw<360 and yaw>225):
         inverse = False
      else:
         inverse = True
      

def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'BAR','outcome2':'outcome4'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome1':'FOO'})
    
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start() 
   
    # Execute SMACH plan
    outcome = sm.execute()
    
   
    sis.stop()
    


if __name__ == '__main__':
    main()