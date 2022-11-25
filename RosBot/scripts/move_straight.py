#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class MoveStraight():
    def __init__(self):
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)
        self.msg = Twist()
        self.velocityValue()

    def velocityValue(self):
        self.msg.linear.x = 1
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.x = 0
        self.msg.angular.y = 0
        self.msg.angular.z = 0

    def publisher(self):
        self.pub.publish(self.msg)
        
def main():
    rospy.init_node("move_straight")
    move_st = MoveStraight()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        move_st.publisher()
        rate.sleep()

if __name__=="__main__":
    main()
