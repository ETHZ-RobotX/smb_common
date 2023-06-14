#!/usr/bin/env python3

# Global
import rospy
import numpy as np
from sensor_msgs.msg import Imu

class PoseNoisifier:

  def __init__(self):

    self.imu_topic_name = "/versavis/imu"

    # ROS
    rospy.init_node("PoseNoisifierNode", anonymous=True)

    # Publisher
    self.imu_publisher = rospy.Publisher("/versavis/imu_noisified", Imu, queue_size=10)

  def subscriber_callback(self, imu):
    print("Received imu message.")
    x_bias = 0.0
    y_bias = 0.0
    z_bias = 0.8
    imu.linear_acceleration.x += x_bias
    imu.linear_acceleration.y += y_bias
    imu.linear_acceleration.z += z_bias
    self.imu_publisher.publish(imu)

  def start_noisifying(self):
    print("======================")
    print("Starting imu noisifying...")
    print("======================")
    rospy.Subscriber(self.imu_topic_name, Imu, self.subscriber_callback)
    rospy.spin()

if __name__ == "__main__":
  
  pose_noisifier = PoseNoisifier()
  pose_noisifier.start_noisifying()