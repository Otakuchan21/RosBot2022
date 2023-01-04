#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry as odom
from tf.transformations import euler_from_quaternion
from RosBot.srv import correctionServiceMessage, correctionServiceMessageResponse
import math

class correction():
    def __init__(self):
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)
        self.sub = rospy.Subscriber('/odom', odom, self.callback)
        self.msg = Twist()
        self.velocityValue()
        self.z_deg = 0
        self.initial_z = 0.0
        self.flag = True

    def velocityValue(self):
        self.msg.linear.x = 0
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.x = 0
        self.msg.angular.y = 0
        self.msg.angular.z = 0
    
    def callback(self, data):
        q_base = data.pose.pose.orientation
        q_list = [q_base.x, q_base.y, q_base.z, q_base.w]
        euler_x, euler_y, euler_z = euler_from_quaternion(q_list)
        self.z_deg = math.degrees(euler_z)
        if self.flag == True:
            self.initial_z = self.z_deg
            print('initial z angle:', self.initial_z)
            self.flag = False
    
    def service_cb(self, request):
        ul = self.initial_z + 5.0
        ll = self.initial_z - 5.0
        r = rospy.Rate(1)
        while not (abs(self.z_deg)< ul and self.z_deg> ll):
            #print(self.z_deg)
            if abs(self.z_deg) > ul:
                self.msg.linear.x = 0
                self.msg.angular.z = -0.5
            elif self.z_deg < ll:
                self.msg.linear.x = 0
                self.msg.angular.z = 0.5
            self.pub.publish(self.msg)
            r.sleep()

        self.msg.angular.z = 0
        self.pub.publish(self.msg)
        r.sleep()
        print('Im in service callback')
        return correctionServiceMessageResponse("reorientation complete", 0)


if __name__=="__main__":
    rospy.init_node("correction")
    o = correction()
    service = rospy.Service('correction', correctionServiceMessage, o.service_cb)
    rospy.spin()
