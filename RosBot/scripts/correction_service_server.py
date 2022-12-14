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

def service_cb(request):
    c = correction()
    while not (abs(c.z_deg)<94.0 and c.z_deg>86.0):
        print(c.z_deg)
        if abs(c.z_deg) > 94.0:
            c.msg.linear.x = 0
            c.msg.angular.z = -0.5
        if c.z_deg < 86.0:
            c.msg.linear.x = 0
            c.msg.angular.z = 0.5
    
    
        c.pub.publish(c.msg)

    c.msg.angular.z = 0
    c.pub.publish(c.msg)
    print('Im in service callback')
    
    return correctionServiceMessageResponse("reorientation complete", 0)

if __name__=="__main__":
    rospy.init_node("correction")
    service = rospy.Service('correction', correctionServiceMessage, service_cb)
    rospy.spin()
