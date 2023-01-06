import rospy
from sensor_msgs.msg import Imu

# Initialize variables to store the current and previous time and velocity of the robot
current_time = None
previous_time = None
current_velocity = [0, 0, 0]
previous_velocity = [0, 0, 0]

# Initialize a variable to store the distance traveled
distance_traveled = 0

# Define the callback function for the IMU message
def imu_callback(msg):
  global current_time, previous_time, current_velocity, previous_velocity, distance_traveled
  
  # Store the current time and acceleration of the robot
  current_time = msg.header.stamp.to_sec()
  current_acceleration = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
  
  # If this is the first message received, set the previous time and velocity to the current values
  if previous_time is None:
    previous_time = current_time
    previous_velocity = current_velocity
  
  # Otherwise, compute the elapsed time and update the velocity using the acceleration
  else:
    dt = current_time - previous_time
    current_velocity = [v + a * dt for v, a in zip(previous_velocity, current_acceleration)]
    
    # Compute the distance traveled as the Euclidean distance between the current and previous velocity
    dx = current_velocity[0] - previous_velocity[0]
    dy = current_velocity[1] - previous_velocity[1]
    dz = current_velocity[2] - previous_velocity[2]
    distance_traveled += (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
    
    # Update the previous time and velocity to the current values
    previous_time = current_time
    previous_velocity = current_velocity

# Initialize the ROS node and subscriber
rospy.init_node('distance_tracker')
sub = rospy.Subscriber('imu', Imu, imu_callback)

# Spin until the node is stopped
rospy.spin()
