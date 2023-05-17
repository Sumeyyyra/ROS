#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Engelin bulunup bulunmadığını kontrol etmek için değişken
engel_var = False

# Lidar verilerini işlemek ve engel olup olmadığını kontrol etmek için fonksiyon
def lidar_callback(msg):
    global engel_var
    # Lidar sensöründen gelen veriler
    ranges = msg.ranges
    # Lidar sensörü önünde bir engel varsa
    if min(ranges) < 0.5:
        engel_var = True
    else:
        engel_var = False

# Engelin etrafında dolaşmak için fonksiyon
def engel_etrafinda_dolas():
    global engel_var

    # ROS node'u oluşturma ve isimlendirme
    rospy.init_node('engel_etrafinda_dolas', anonymous=True)

    # Twist mesajını kullanarak TurtleBot3'ü hareket ettirmek için bir yayıncı oluşturma
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    # Lidar verilerini okumak için bir abone oluşturma
    rospy.Subscriber("scan", LaserScan, lidar_callback)

    # Hareket mesajları için Twist mesajını tanımlama
    hareket = Twist()

    # Robotun ilerlemesi için hızlar
    hareket.linear.x = 0.1  # 0.1 m/s hızda ilerlesin
    hareket.angular.z = 0.2  # 0.2 rad/s hızda dönsün

    # Hareket etmeye başlama
    pub.publish(hareket)

    while not rospy.is_shutdown():
        if engel_var:
            # Engelden kaçınmak için hareket etmek için yeni hızlar
            hareket.linear.x = 0.5 # ileri gitmeyi durdur
            hareket.angular.z = 0.2 # dönme hızını artır

        else:
            # Engel yoksa, normal hızlara geri dön
            hareket.linear.x = 0.1
            hareket.angular.z = 0.2

        pub.publish(hareket)

# Ana işlem
if __name__ == '__main__':
    try:
        engel_etrafinda_dolas()
    except rospy.ROSInterruptException:
        pass