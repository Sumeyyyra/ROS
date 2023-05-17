#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import smach 
import smach_ros
import time
import numpy as np
from geometry_msgs.msg import Twist

class LaneFollowingState(smach.State):
    def __init__(self):
        rospy.init_node("serit_takip")
        self.bridge = CvBridge()
        rospy.Subscriber("camera/rgb/image_raw",Image,self.kameraCallback)
        self.pub = rospy.Publisher("cmd_vel",Twist,queue_size = 10)
        self.hiz_mesaji = Twist()
        rospy.spin()
    def kameraCallback(self,data):
        print("aa")
        img = self.bridge.imgmsg_to_cv2(data,"bgr8")
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
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(img,(cx,cy),5,(255,0,0),-1)
            sapma = cx - w/2
            self.hiz_mesaji.linear.x = 0.2
            self.hiz_mesaji.angular.z = -sapma/100
            self.pub.publish(self.hiz_mesaji)
        else:
            self.hiz_mesaji.linear.x = 0.0
            self.hiz_mesaji.angular.z = 0.0
            self.pub.publish(self.hiz_mesaji)
        
       
        # Burada şerit takibi işlemi yapılır
        # cv_image üzerinde görüntü işleme teknikleri kullanarak şerit takibi yapılır

    def execute(self, userdata):
        rospy.loginfo('Executing state LANE_FOLLOWING')
        #while not rospy.is_shutdown():
           # LaneFollowingState()
            # Burada şerit takibi işlemi devamlı olarak yapılır
        time.sleep(0.1)
            
            # Şerit takibi işlemi sonucu engel algılandıysa, 'obstacle_detected' çıkışı verilir
            # Şerit takibi işlemi devam ederse, döngü devam eder


class ObstacleDetectedState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['obstacle_cleared', 'exit'])
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

  
    def image_callback(self, data):
    # Burada engel algılama ve engeli aşma işlemi yapılır
    # cv_image üzerinde görüntü işleme teknikleri kullanarak engel algılama ve engeli aşma işlemi yapılır

    # Eğer sol tarafın yoğunluğu sağ tarafın yoğunluğundan büyükse, engel sol tarafta demektir
        if 4<5:
            # engel sol tarafta, engeli aşmak için gerekli hareketler yapılır
            pass
        else:
            # engel sağ tarafta, engeli aşmak için gerekli hareketler yapılır
            pass

def main():
    rospy.init_node('lane_following_fsm')
    print("a")
    # FSM tanımlanır ve durumları eklenir
    lane_following_fsm = smach.StateMachine(outcomes=['exit'])
    with lane_following_fsm:
        smach.StateMachine.add('LANE_FOLLOWING', LaneFollowingState(), 
                               transitions={'obstacle_detected': 'OBSTACLE_DETECTED', 'exit': 'exit'})
        smach.StateMachine.add('OBSTACLE_DETECTED', ObstacleDetectedState(), 
                               transitions={'obstacle_cleared': 'LANE_FOLLOWING', 'exit': 'exit'})
    
    # FSM çalıştırılır
    # burda bi server yaynılaman lazımdı onu  ekledik sisle
    sis = smach_ros.IntrospectionServer('server_name', lane_following_fsm, '/SM_ROOT')
    sis.start()
    outcome = lane_following_fsm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()