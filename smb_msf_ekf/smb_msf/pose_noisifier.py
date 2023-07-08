#!/usr/bin/env python3

# Global
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped

class PoseNoisifier:

  def __init__(self):

    self.transform_topic_name = "/mapping_node/scan2map_transform"

    # ROS
    rospy.init_node("PoseNoisifierNode", anonymous=True)

    # Publisher
    self.transform_publisher = rospy.Publisher("/mapping_node/scan2map_transform_noisified", TransformStamped, queue_size=10)

  def subscriber_callback(self, transform):
    print("Received transform message.")
    gaussian_noise = np.random.normal(0.0, 0.1, size=3)
    transform.transform.translation.x += gaussian_noise[0]
    transform.transform.translation.y += gaussian_noise[1]
    transform.transform.translation.z += gaussian_noise[2]
    self.transform_publisher.publish(transform)

  def start_noisifying(self):
    print("======================")
    print("Starting pose noisifying...")
    print("======================")
    rospy.Subscriber(self.transform_topic_name, TransformStamped, self.subscriber_callback)
    rospy.spin()

if __name__ == "__main__":
  
  pose_noisifier = PoseNoisifier()
  pose_noisifier.start_noisifying()