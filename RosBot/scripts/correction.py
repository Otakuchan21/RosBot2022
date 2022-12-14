#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry as odom
from tf.transformations import euler_from_quaternion
from RosBot.srv import correction, correctionResponse
import math

class correction():
    def __init__(self):
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)
        self.msg = Twist()
        self.velocityValue()
        self.z_deg = 0

    def velocityValue(self):
        self.msg.linear.x = 0
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.x = 0
        self.msg.angular.y = 0
        self.msg.angular.z = 0
    
    def callback(self,data):
        q_base = data.pose.pose.orientation
        q_list = [q_base.x, q_base.y, q_base.z, q_base.w]
        euler_x, euler_y, euler_z = euler_from_quaternion(q_list)
        self.z_deg = math.degrees(euler_z)
    
    def service_cb(self, request):
        while self.z_deg!<95.0 and self.z_deg!>85.0:
            sub = rospy.Subscriber('pose', odom, self.callback)
            if self.z_deg > 95.0 or self.z_deg < 85.0:
                if z_deg > 95.0:
                    self.msg.linear.x = 0
                    self.msg.angular.z = -0.5
                if z_deg < 85.0:
                    self.msg.linear.x = 0
                    self.msg.angular.z = 0.5
        
            pub.publish(self.msg)
        
        return correctionResponse("reorientation complete", 0)
        

        
        
def main():
    rospy.init_node("correction")
    corr = correction()
    service = rospy.Service('correction', correction, corr.service_cb)
    rospy.spin()

if __name__=="__main__":
    main()
