#!/usr/bin/env python3

import rospy
import smach
import smach_ros

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class FollowLine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['obstacle_detected', 'preempted'])
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.laser_sub = rospy.Subscriber('scan', LaserScan, self.laser_cb)

    def execute(self, userdata):
        twist = Twist()
        twist.linear.x = 0.2

        while not rospy.is_shutdown():
            if self.is_obstacle_detected():
                return 'obstacle_detected'
            else:
                self.cmd_vel_pub.publish(twist)
        
        return 'preempted'

    def laser_cb(self, msg):
        self.ranges = msg.ranges

    def is_obstacle_detected(self):
        min_range = min(self.ranges)
        if min_range < 0.5:
            return True
        else:
            return False

class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_following', 'preempted'])
        self.wait_time = rospy.Duration.from_sec(15)

    def execute(self, userdata):
        rospy.sleep(self.wait_time)
        return 'continue_following'

class Navigate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_following', 'preempted'])

    def execute(self, userdata):
        # implement navigation logic here
        return 'continue_following'

if __name__ == '__main__':
    rospy.init_node('follow_line')

    sm = smach.StateMachine(outcomes=['preempted'])
    with sm:
        smach.StateMachine.add('FOLLOW_LINE', FollowLine(),
                               transitions={'obstacle_detected':'WAIT',
                                            'preempted':'preempted'})
        smach.StateMachine.add('WAIT', Wait(),
                               transitions={'continue_following':'NAVIGATE',
                                            'preempted':'preempted'})
        smach.StateMachine.add('NAVIGATE', Navigate(),
                               transitions={'continue_following':'FOLLOW_LINE',
                                            'preempted':'preempted'})

    sis = smach_ros.IntrospectionServer('follow_line_server', sm, '/FOLLOW_LINE')
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()