#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry as odom
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from RosBot.srv import correctionServiceMessage, correctionServiceMessageResponse
import math
z_deg = 0
class correction():
    def __init__(self):
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)
        self.sub = rospy.Subscriber('/imu', Imu, self.callback)
        self.msg = Twist()
        self.velocityValue()

    def velocityValue(self):
        self.msg.linear.x = 0
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.x = 0
        self.msg.angular.y = 0
        self.msg.angular.z = 0
    
    def callback(self,data):
        global z_deg
        r = rospy.Rate(10)
        q_base = data.orientation
        q_list = [q_base.x, q_base.y, q_base.z, q_base.w]
        euler_x, euler_y, euler_z = euler_from_quaternion(q_list)
        z_deg = abs(math.degrees(euler_z))
        print('im in subscriber callback')
        print(z_deg)
        r.sleep()

def service_cb(request):
    c = correction()
    r = rospy.Rate(1)
    global z_deg
    
    while not (z_deg<99.0 and z_deg>85.0):
        #print(z_deg)
        if z_deg > 94.0:
            c.msg.linear.x = 0
            c.msg.angular.z = -0.5
        if z_deg < 86.0:
            c.msg.linear.x = 0
            c.msg.angular.z = 0.5
        c.pub.publish(c.msg)
        r.sleep()
        print(c.msg.angular.z)
    
    c.msg.angular.z = 0
    if c.msg.angular.z ==0:
        c.pub.publish(c.msg)
        r.sleep()
        print('im in 0')
    return correctionServiceMessageResponse("reorientation complete", 0)


rospy.init_node("correction")
service = rospy.Service('correction', correctionServiceMessage, service_cb)
rospy.spin()
