#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from RosBot.srv import correctionServiceMessage as csm

class avoidance():
    def __init__(self):
        self.Laser_sub = rospy.Subscriber('/scan', LaserScan, self.SectorScan)
        self.IRL_sub = rospy.Subscriber('/range/fl', Range, self.IrFrontLeft)
        self.IRR_sub = rospy.Subscriber('/range/fr', Range, self.IrFrontRight)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.counter = 0
        self.normal_linear = 0.2
        # obstacles are detected if they are within this range
        self.detection_dist = 0.3
        # while avoiding an obstacle the rosbot will move back to be able to rotate
        self.avoid_linear = -0.15
        self.avoid_angular = 1.1 
        self.front_left = False
        self.front_right = False
        # divides the rplider scan into 12 section and keeps track of their order, measured distances and deviation costs
        self.sector_angle = 30
        self.sectors = ["front_C", "front_L", "left_R", "left_C", "left_L", "back_R", "back_C", "back_L", "right_R", "right_C", "right_L", "front_R"]
        self.sector_distances = {"front_C":[], "front_L":[], "left_R":[], "left_C":[], "left_L":[], "back_R":[], "back_C":[], "back_L":[], "right_R":[], "right_C":[], "right_L":[], "front_R":[] }


    def IrFrontLeft(self, scan):
        print(scan.range)
        x = scan.range
        if (x <= self.detection_dist and x != 'inf'):
            self.front_left = True
        else:
            self.front_left = False
            #sector_distances["front_L"] = [x]
        
    def IrFrontRight(self, scan):
        print(scan.range)
        x = scan.range
        if (x <= self.detection_dist and x != 'inf'):
            self.front_right = True
        else:
            self.front_right = False
            #sector_distances["front_L"] = [x]

    def SectorScan(self, scan):

        # takes in the scan data and records the distance to an obstacle if it detects one
        #looks at the data scanned within the range of each sector
        for i, sector in enumerate(self.sectors):
            self.sector_distances[sector] = [ x for x in scan.ranges[self.sector_angle*i : self.sector_angle*(i+1)] 
                                            if x <= self.detection_dist and x != 'inf']        #records the distance to the obstacle if it is within the predefined collision range and not infinite
        if (self.front_left == True):
            print('reachedl')
            self.sector_distances["front_L"] = [0.2]

        if (self.front_right == True):
            print('reachedr')
            self.sector_distances["front_R"] = [0.2]

    def action(self):
        msg = Twist()
        r = rospy.Rate(1)
        linear_x = 0
        angular_z = 0
        description = ""
        front_obstacle = len(self.sector_distances["front_C"])
        Rfront_obstacle = len(self.sector_distances["front_R"])
        Lfront_obstacle =  len(self.sector_distances["front_L"])
        #print(sector)
        logmessage = {description: angular_z}

        if not front_obstacle and not Rfront_obstacle and not Lfront_obstacle:
            description = "no obstacle - move straight"
            linear_x = self.normal_linear
            angular_z = 0
            self.counter +=1
        elif front_obstacle and not Rfront_obstacle and not Lfront_obstacle:
            description = "front obstacle - left/right clear"
            linear_x = self.avoid_linear
            angular_z = self.avoid_angular 
            self.counter = 0
        elif not front_obstacle and not Rfront_obstacle and Lfront_obstacle:
            description = "left obstacle - front/right clear"
            linear_x = self.avoid_linear
            angular_z = -self.avoid_angular
            self.counter = 0
        elif not front_obstacle and Rfront_obstacle and not Lfront_obstacle:
            description = "right obstacle - front/left clear"
            linear_x = self.avoid_linear
            angular_z = self.avoid_angular
            self.counter = 0
        elif front_obstacle and Rfront_obstacle and not Lfront_obstacle:
            description = "left clear - front/right obstacle"
            linear_x = self.avoid_linear
            angular_z = self.avoid_angular 
            self.counter = 0
        elif front_obstacle and not Rfront_obstacle and Lfront_obstacle:
            description = "right clear - front/left obstacle"
            linear_x = self.avoid_linear
            angular_z = -self.avoid_angular
            self.counter = 0
        elif not front_obstacle and Rfront_obstacle and Lfront_obstacle:
            description = "front clear - left/right obstacle"
            linear_x = self.avoid_linear
            angular_z = self.avoid_angular
            self.counter = 0
        elif front_obstacle and Rfront_obstacle and Lfront_obstacle:
            description = "none clear"
            linear_x = self.avoid_linear
            angular_z = self.avoid_angular
            self.counter = 0
        else:
            description = "unknown"
            linear_x = 0.0
            angular_z = 0.0
            self.counter = 0
        
        if self.counter>50:
            #call service
            rospy.wait_for_service('correction')
            service_requester = rospy.ServiceProxy('correction', csm)
            message = 'initiate orientation sequence'
            
            response = service_requester(message)
            self.counter = response.counter
        
        logmessage = {description: angular_z}
        rospy.loginfo(logmessage)
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.pub.publish(msg)
        r.sleep()

def main():
    rospy.init_node('avoidance')
    rate = rospy.Rate(5)
    obj = avoidance()
    while not rospy.is_shutdown():
        obj.action()

if __name__ == '__main__':
    main()
