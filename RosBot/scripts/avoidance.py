#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

normal_linear = 0.2

# obstacles are detected if they are within this range
detection_dist = 0.3

# while avoiding an obstacle the rosbot will move back to be able to rotate
avoid_linear = -0.1
avoid_angular = 1.5

# divides the rplider scan into 12 section and keeps track of their order, measured distances and deviation costs
sector_angle = 30
sectors = ["front_C", "front_L", "left_R", "left_C", "left_L", "back_R","back_C", "back_L", "right_R", "right_C", "right_L", "front_R"]
sector_distances = {"front_C":[], "front_L":[], "left_R":[], "left_C":[], "left_L":[], "back_R":[], "back_C":[], "back_L":[], "right_R":[], "right_C":[], "right_L":[], "front_R":[]}
sector_costs = {"front_C": 0, "front_L": 1, "left_R": 2, "left_C": 3, "left_L": 4, "back_R": 5, "back_C": 6, "back_L": -5, "right_R": -4, "right_C": -3, "right_L": -2, "front_R": -1}

def SectorScan(scan):
    # takes in the scan data and records the distance to an obstacle if it detects one
    for i, sector in enumerate(sectors):
        print ("scanning in sector ", sector)
        sector_distances[sector] = [ x for x in scan.ranges[sector_angle*i : sector_angle*(i+1)] #looks at the data scanned within the range of each sector
                                        if x <= detection_dist and x != 'inf']        #records the distance to the obstacle if it is within the predefined collision range and not infinite

def ClearestPath(velocity):
        # orients the rosbot so it wants to move towards the front and center
        goal = "front_C"
        closest_dist = 10e6
        sector_cost = 0
        best_sector = {"destination": "back_C", "distance": 10e-6}

        # for each sector it checks if the path is clear and the cost to determine the orientation
        for sector in sector_distances.items():
            sector_cost = abs(sector_costs[sector[0]]-sector_costs[goal])
            # if an obstacle is detected we find a clearer path
            if not len(sector[1]):
                #checks if it's the cheapest path
                if (sector_cost < closest_dist):
                    closest_dist = sector_cost
                    best_sector["distance"] = detection_dist
                    best_sector["destination"] = sector[0]
            # check if it's the clearest path
            elif(max(sector[1]) > best_sector["distance"]):
                best_sector["distance"] = max(sector[1])
                best_sector["destination"] = sector[0]

        # calculate the cost to the chosen orientation
        sector_cost = sector_costs[best_sector["destination"]]-sector_costs[goal]

        # we change orientation whenever the clearest path is not forwards
        if (closest_dist!=0):
            angular_velocity = ((sector_cost/max(1, abs(sector_cost)))*avoid_angular)
            velocity = Steering(velocity, angular_velocity)
        else:
            velocity = MoveStraight(velocity,normal_linear)
        
        return velocity

# allows the robot to change its orientation to avoid collision
def Steering(velocity, angular_velocity=0):
    velocity.linear.x = avoid_linear
    velocity.linear.y = 0
    velocity.linear.z = 0
    velocity.angular.x = 0
    velocity.angular.y = 0
    velocity.angular.z = angular_velocity
    print ("steer")
    return velocity

# 
def MoveStraight(velocity, normal_linear):
    velocity.linear.x = normal_linear
    velocity.linear.y = 0
    velocity.linear.z = 0
    velocity.angular.x = 0
    velocity.angular.y = 0
    velocity.angular.z = 0
    print("straight")
    return velocity

def main():
    rospy.init_node("avoidance")
    rospy.Subscriber("/scan", LaserScan,SectorScan)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    vel = Twist()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        vel = ClearestPath(vel)
        pub.publish(vel)
        rate.sleep()

if __name__=="__main__":
    main()
