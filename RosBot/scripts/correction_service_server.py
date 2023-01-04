#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
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
        self.initial_z = 0.0
        self.flag = True

    def velocityValue(self):
        self.msg.linear.x = 0
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.x = 0
        self.msg.angular.y = 0
        self.msg.angular.z = 0
    
    def callback(self,data):
        global z_deg
        
        q_base = data.orientation
        q_list = [q_base.x, q_base.y, q_base.z, q_base.w]
        euler_x, euler_y, euler_z = euler_from_quaternion(q_list)
        z_deg = math.degrees(euler_z)
        #print('im in subscriber callback')
        if self.flag == True:
            self.inital_z = z_deg
            self.flag = False
        
        

def service_cb(request):
    global z_deg
    c = correction()
    r = rospy.Rate(1)
    ul = c.initial_z + 5
    ll = c.initial_z - 5
    if z_deg<ul and z_deg>ll:
        return correctionServiceMessageResponse("reorientation not needed", 0)
    else:
        while not (z_deg<ul and z_deg>ll):
            print(z_deg)
            if z_deg > ul or z_deg < 0:
                c.msg.linear.x = 0
                c.msg.angular.z = -0.5
            elif z_deg < ll:
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
