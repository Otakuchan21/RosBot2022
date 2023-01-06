import rospy
from nav_msgs.msg import Odometry

class TimeTracker:
    def __init__(self):
        # Initialize the time tracking variables
        self.moving_time = 0
        self.stopped_time = 0
        self.is_moving = False

        # Subscribe to the odometry topic
        rospy.Subscriber('odom', Odometry, self.odometry_callback)

    def odometry_callback(self, odometry):
        # Check if the robot is moving or stopped
        if odometry.twist.twist.linear.x == 0 and odometry.twist.twist.angular.z == 0:
            if self.is_moving:
                self.stopped_time += rospy.get_rostime() - self.last_time
                self.is_moving = False
        else:
            if not self.is_moving:
                self.last_time = rospy.get_rostime()
                self.is_moving = True

    def print_times(self):
        # Print the time that the robot has been moving and stopped
        rospy.loginfo("Moving time: %s seconds", self.moving_time)
        rospy.loginfo("Stopped time: %s seconds", self.stopped_time)

if __name__ == '__main__':
    rospy.init_node('time_tracker')
    time_tracker = TimeTracker()
    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        time_tracker.print_times()
        rate.sleep()
