#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion
import math

class correction():
    def __init__(self):
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)
        self.msg = Twist()
        self.velocityValue()

    def velocityValue(self):
        self.msg.linear.x = 0.5
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.x = 0
        self.msg.angular.y = 0
        self.msg.angular.z = 0

    def decayFunc(self,error):
        pass
    
    def callback(self,data):
        q_base = data.pose.pose.orientation
        q_list = [q_base.x, q_base.y, q_base.z, q_base.w]
        euler_x, euler_y, euler_z = euler_from_quaternion(q_list)
        z_deg = math.degrees(euler_z)
        if z_deg > 92.0 or z_deg < 88.0:
            if z_deg > 92.0:
                self.msg.linear.x = 0
                self.msg.angular.z = -0.5
            if z_deg < 88.0:
                self.msg.linear.x = 0
                self.msg.linear.z = 0.5
        else:
            velocityValue()

        pub.publish(self.msg)
        

        
        
def main():
    rospy.init_node("correction")
    corr = correction()
    sub = rospy.Subscriber('pose', PoseStamped,corr.callback)
    rate = rospy.Rate(1)
    rospy.spin()

if __name__=="__main__":
    main()
