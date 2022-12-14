#!/usr/bin/env python

import rospy
from RosBot.srv import correction

rospy.init_node('correction_client')
rospy.wait_for_service('correction')
request = rospy.ServiceProxy('correction', correction)

message = 'initiate orientation sequence'

response, counter = request(message)

print counter, response





