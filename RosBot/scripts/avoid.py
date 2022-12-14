#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

normal_linear = 0.4

# obstacles are detected if they are within this range
detection_dist = 0.3

# while avoiding an obstacle the rosbot will move back to be able to rotate
avoid_linear = -0.1
avoid_angular = 0.5 

# divides the rplider scan into 12 section and keeps track of their order, measured distances and deviation costs
sector_angle = 60
sectors = ["front_C", "front_L", "left_R", "left_C", "left_L", "back_R", "back_C", "back_L", "right_R", "right_C", "right_L", "front_R"]
sector_distances = {"front_C":[], "front_L":[], "left_R":[], "left_C":[], "left_L":[], "back_R":[], "back_C":[], "back_L":[], "right_R":[], "right_C":[], "right_L":[], "front_R":[] }

def SectorScan(scan):
    print("scan samples: " + scan.samples)
    print("scan range: " + len(scan.range))
    # takes in the scan data and records the distance to an obstacle if it detects one
    #looks at the data scanned within the range of each sector
    for i, sector in enumerate(sectors):
        sector_distances[sector] = [ x for x in scan.ranges[sector_angle*i : sector_angle*(i+1)] 
                                        if x <= detection_dist and x != 'inf']        #records the distance to the obstacle if it is within the predefined collision range and not infinite
    action(sector_distances)

def action(sector):
    msg = Twist()
    linear_x = 0
    angular_z = 0
    description = ""
    front_obstacle = len(sector["front_C"])
    Rfront_obstacle = len(sector["back_C"])
    Lfront_obstacle =  len(sector["front_L"])

    logmessage = {description: angular_z}

    if not front_obstacle and not Rfront_obstacle and not Lfront_obstacle:
        description = "no obstacle - move straight"
        linear_x = normal_linear
        angular_z = 0
    elif front_obstacle and not Rfront_obstacle and not Lfront_obstacle:
        description = "front obstacle - left/right clear"
        linear_x = avoid_linear
        angular_z = avoid_angular
    elif not front_obstacle and not Rfront_obstacle and Lfront_obstacle:
        description = "left obstacle - front/right clear"
        linear_x = avoid_linear
        angular_z = avoid_angular if angular_z > 0 else -avoid_angular
    elif not front_obstacle and Rfront_obstacle and not Lfront_obstacle:
        description = "right obstacle - front/left clear"
        linear_x = avoid_linear
        angular_z = -avoid_angular
    elif front_obstacle and Rfront_obstacle and not Lfront_obstacle:
        description = "left clear - front/right obstacle"
        linear_x = avoid_linear
        angular_z = -avoid_angular 
    elif front_obstacle and not Rfront_obstacle and Lfront_obstacle:
        description = "right clear - front/left obstacle"
        linear_x = avoid_linear
        angular_z = avoid_angular
    elif not front_obstacle and Rfront_obstacle and Lfront_obstacle:
        description = "front clear - left/right obstacle"
        linear_x = avoid_linear
        angular_z = avoid_angular
    elif front_obstacle and Rfront_obstacle and Lfront_obstacle:
        description = "none clear"
        linear_x = avoid_linear
        angular_z = avoid_angular
    else:
        description = "unknown"
        linear_x = 0.0
        angular_z = 0.0
    
    logmessage = {description: angular_z}
    rospy.loginfo(logmessage)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)

def main():
    global pub

    rospy.init_node('avoidance')
    rospy.Subscriber('/scan', LaserScan, SectorScan)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    main()