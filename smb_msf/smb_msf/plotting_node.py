#!/usr/bin/env python3

# Global
import numpy
import matplotlib
import rospy

# Custom
from msf_core.msg import DoubleArrayStamped


# TODO: Add a main that plots the trajectory and covariance of the estimation.

class MsfPlotter:

  def __init__(self):

    self.state_topic_name = "/msf_core/state_out"

    # ROS
    rospy.init_node('MSF Publisher Node', anonymous=True)


  def subscriber_callback(self, state_estimate):
    print("Received state message.")


  def start_plotting(self):

    print("Starting plotting...")
    rospy.Subscriber(self.state_topic_name, DoubleArrayStamped,
                         self.subscriber_callback)

    rospy.spin()



if __name__ == "__main__":
  
  msf_plotter = MsfPlotter()
  msf_plotter.start_plotting()