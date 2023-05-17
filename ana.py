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
        smach.State.__init__(self, outcomes=['obstacle_detected', 'exit'])
        self.bridge = CvBridge()
        #self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        rospy.Subscriber("camera/rgb/image_raw",Image,self.image_callback)
        self.pub = rospy.Publisher("cmd_vel",Twist,queue_size = 10)
        self.hiz_mesaji = Twist()
        rospy.spin()
        
    def image_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        alt_sari = np.array([20,100,100])
        ust_sari = np.array([40,100,100])
        maske = cv2.inRange(hsv,alt_sari,ust_sari)
        sonuc = cv2.bitwise_and(cv_image,cv_image,mask=maske)
        cv2.imshow("1",cv_image)
        cv2.imshow("2",maske)
        cv2.imshow("3",sonuc)
        
        # Burada şerit takibi işlemi yapılır
        # cv_image üzerinde görüntü işleme teknikleri kullanarak şerit takibi yapılır

    def execute(self, data):
        rospy.loginfo('Executing state LANE_FOLLOWING')
        while True:
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
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # Burada engel algılama ve engeli aşma işlemi yapılır
        # cv_image üzerinde görüntü işleme teknikleri kullanarak engel algılama ve engeli aşma işlemi yapılır

    def execute(self, data):
        rospy.loginfo('Executing state OBSTACLE_DETECTED')
        start_time = time.time()
        while True:
            time.sleep(0.1)
            # Engelin hala var olduğu kontrol edilir
            if time.time() - start_time >= 15:
                # Engel 15 saniyeden uzun süre varsa, çıkış verilir
                return 'exit'
            # Engel algılanmadıysa, 'obstacle_cleared' çıkışı verilir
            # Engel algılandıysa, döngü devam eder


def main():
    rospy.init_node('lane_following_fsm')

    # FSM tanımlanır ve durumları eklenir
    lane_following_fsm = smach.StateMachine(outcomes=['exit'])
    with lane_following_fsm:
        smach.StateMachine.add('LANE_FOLLOWING', LaneFollowingState(), 
                               transitions={'obstacle_detected': 'OBSTACLE_DETECTED', 'exit': 'exit'})
        smach.StateMachine.add('OBSTACLE_DETECTED', ObstacleDetectedState(), 
                               transitions={'obstacle_cleared': 'LANE_FOLLOWING', 'exit': 'exit'})  
    sis = smach_ros.IntrospectionServer('server_name', lane_following_fsm, '/SM_ROOT')
    sis.start()
    # FSM çalıştırılır
    outcome = lane_following_fsm.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()