import rospy
from sensor_msgs.msg import Imu

# Threshold for determining if the ROSbot2 is moving or stopped
MOVEMENT_THRESHOLD = 0.1  # Set this value based on your needs

# Variables to track the time while moving and stopped
time_moving = 0
time_stopped = 0

def imu_callback(imu_msg):
    global time_moving, time_stopped

    # Get the current time in seconds
    current_time = rospy.Time.now().to_sec()

    # Check if the ROSbot2 is moving or stopped based on the linear acceleration data
    if abs(imu_msg.linear_acceleration.x) > MOVEMENT_THRESHOLD or \
       abs(imu_msg.linear_acceleration.y) > MOVEMENT_THRESHOLD or \
       abs(imu_msg.linear_acceleration.z) > MOVEMENT_THRESHOLD:
        # ROSbot2 is moving
        time_moving += current_time - previous_time
    else:
        # ROSbot2 is stopped
        time_stopped += current_time - previous_time

    # Save the current time for the next iteration
    previous_time = current_time

def main():
    global previous_time

    # Initialize the ROS node
    rospy.init_node("imu_time_tracker")

    # Subscribe to the IMU topic
    rospy.Subscriber("/imu", Imu, imu_callback)

    # Set the previous time to the current time
    previous_time = rospy.Time.now().to_sec()

    # Spin to keep the node alive and receive IMU messages
    rospy.spin()

if __name__ == "__main__":
    main()
