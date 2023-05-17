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
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

    def image_callback(self, data):
       # cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cap = cv2.VideoCapture(0)
        cap = cv2.VideoCapture('video.mp4')
        while True:
        # Kameradan / video dosyasından bir kare okuyun
            ret, frame = cap.read()
    
        # Kare okunamadıysa, döngüden çıkın
        if not ret:
            break
    
        # Kareyi siyah beyaz formata dönüştürün
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
        # Beyaz şeridi belirlemek için bir eşik değeri belirleyin
        thresh = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)[1]
    
        # Beyaz şeridi bulun
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Beyaz şerit bulundu, bunu takip edin
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
    
        # İşlenmiş kareyi gösterin
        cv2.imshow('frame', frame)
    
        # 'q' tuşuna basıldığında döngüyü kırın
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Kamerayı serbest bırakın ve tüm pencereleri kapatın
    cap.release()
    cv2.destroyAllWindows()
        # Burada şerit takibi işlemi yapılır
        # cv_image üzerinde görüntü işleme teknikleri kullanarak şerit takibi yapılır

    def execute(self, userdata):
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

    def execute(self, userdata):
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