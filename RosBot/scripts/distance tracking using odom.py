import rospy
from nav_msgs.msg import Odometry

# Initialize variables to store the current and previous position of the robot
current_position = None
previous_position = None

# Initialize a variable to store the distance traveled
distance_traveled = 0

# Define the callback function for the odometry message
def odometry_callback(msg):
  global current_position, previous_position, distance_traveled
  
  # Store the current position of the robot
  current_position = msg.pose.pose.position
  
  # If this is the first message received, set the previous position to the current position
  if previous_position is None:
    previous_position = current_position
  
  # Otherwise, compute the distance traveled as the Euclidean distance between the current and previous position
  else:
    dx = current_position.x - previous_position.x
    dy = current_position.y - previous_position.y
    distance_traveled += (dx ** 2 + dy ** 2) ** 0.5
    
    # Update the previous position to the current position
    previous_position = current_position

# Initialize the ROS node and subscriber
rospy.init_node('distance_tracker')
sub = rospy.Subscriber('odom', Odometry, odometry_callback)

# Spin until the node is stopped
rospy.spin()
