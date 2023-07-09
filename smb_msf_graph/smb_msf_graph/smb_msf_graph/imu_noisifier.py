#!/usr/bin/env python3

# Global
import rospy
import numpy as np
from sensor_msgs.msg import Imu

class ImuNoisifier:

  def __init__(self):

    self.imu_topic_name = "/imu"

    # Flags
    self.is_first_imu = True

    # ROS
    rospy.init_node("PoseNoisifierNode", anonymous=True)

    # Publisher
    self.imu_publisher = rospy.Publisher("/imu/noisified", Imu, queue_size=10)

    # Bias Amplitude
    self.x_bias = 0.0
    self.y_bias = 0.0
    self.z_bias = 0.1

    # Initial Time
    self.initial_time = []

    # Period Duration
    self.period_duration = 20.0

  def subscriber_callback(self, imu):
    if self.is_first_imu:
      print("Received imu message.")
      self.is_first_imu = False
      self.initial_time = imu.header.stamp.to_sec()

    imu.linear_acceleration.x += self.x_bias * np.sin(2 * np.pi / self.period_duration * (imu.header.stamp.to_sec() - self.initial_time))
    imu.linear_acceleration.y += self.y_bias * np.sin(2 * np.pi / self.period_duration * (imu.header.stamp.to_sec() - self.initial_time))
    imu.linear_acceleration.z += self.z_bias * np.sin(2 * np.pi / self.period_duration * (imu.header.stamp.to_sec() - self.initial_time))
    self.imu_publisher.publish(imu)

  def start_noisifying(self):
    print("======================")
    print("Starting imu noisifying...")
    print("======================")
    rospy.Subscriber(self.imu_topic_name, Imu, self.subscriber_callback)
    rospy.spin()

if __name__ == "__main__":
  
  imu_noisifier = ImuNoisifier()
  imu_noisifier.start_noisifying()