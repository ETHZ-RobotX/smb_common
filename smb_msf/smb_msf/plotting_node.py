#!/usr/bin/env python3

# Global
import numpy as np
import matplotlib.pyplot as plt
import rospy
import os
import time

# Custom
from msf_core.msg import DoubleArrayStamped

experiment_name = ""


def plot_2d(x_coords: list, y_coords: list, x_axis_name: str, y_axis_name: str, plot_name: str):
  # Check length
  print("Plot: " + plot_name + ". Length of " + x_axis_name + ": " + str(len(x_coords)) + ". Length of " + y_axis_name + ": " + str(len(y_coords)))
  assert np.abs(len(x_coords) - len(y_coords)) <= 1
  if len(x_coords) - len(y_coords) == 1:
    x_coords.pop() # remove last element
  elif len(y_coords) - len(x_coords) == 1:
    y_coords.pop()
  # Home-dir
  home_dir = os.path.expanduser('~')
  # Plot
  plt.rcParams['lines.markersize'] = 0.5
  plt.rcParams["figure.figsize"] = (6,6)
  plt.xticks(fontsize=24)
  plt.yticks(fontsize=24)
  fig, ax = plt.subplots(1,1)
  ax.set(xlabel=x_axis_name, ylabel=y_axis_name, title=plot_name)
  ax.grid()
  plt.tight_layout()
  ax.plot(x_coords, y_coords, c="green", linewidth=0.7, alpha=1.0)
  fig.savefig(os.path.join(home_dir, "rss_plots", experiment_name + "_" + plot_name + ".png"))


class MsfPlotter:

  def __init__(self):

    self.state_topic_name = "/msf_core/state_out"

    # ROS
    rospy.init_node('MSF Publisher Node', anonymous=True)

    # Create dir
    home_dir = os.path.expanduser('~')
    if not os.path.exists(os.path.join(home_dir, "rss_plots")):
      os.makedirs(os.path.join(home_dir, "rss_plots"))

    # Stored messages
    ## Time
    self.initial_time = []
    self.time = []
    ## Position
    self.x_pos = []
    self.y_pos = []
    self.z_pos = []
    ## Velocity
    self.x_vel = []
    self.y_vel = []
    ## Acc Bias
    self.x_acc_bias = []
    self.y_acc_bias = []
    self.z_acc_bias = []

  def plot(self):

    # Plot diagrams when destructor is called
    print("Saving plots...")
    ## x-y-plot
    plot_name = "position-x-y"
    print(plot_name)
    x_axis_name = "x[m]"
    y_axis_name = "y[m]"
    plot_2d(x_coords=self.x_pos, y_coords=self.y_pos, x_axis_name=x_axis_name, y_axis_name=y_axis_name, plot_name=plot_name)
    ## z-plot
    plot_name = "position-z"
    print(plot_name)
    x_axis_name = "t[s]"
    y_axis_name = "z[m]"
    plot_2d(x_coords=self.time, y_coords=self.z_pos, x_axis_name=x_axis_name, y_axis_name=y_axis_name, plot_name=plot_name)
    ## x-velocity
    plot_name = "velocity-x"
    print(plot_name)
    x_axis_name = "t[s]"
    y_axis_name = "v[m/s]"
    plot_2d(x_coords=self.time, y_coords=self.x_vel, x_axis_name=x_axis_name, y_axis_name=y_axis_name, plot_name=plot_name)
    ## y-velocity
    plot_name = "velocity-y"
    print(plot_name)
    x_axis_name = "t[s]"
    y_axis_name = "v[m/s]"
    plot_2d(x_coords=self.time, y_coords=self.y_vel, x_axis_name=x_axis_name, y_axis_name=y_axis_name, plot_name=plot_name)
    ## x-Bias
    plot_name = "acc-bias-x"
    print(plot_name)
    x_axis_name = "t[s]"
    y_axis_name = "b[m/s^2]"
    plot_2d(x_coords=self.time, y_coords=self.x_acc_bias, x_axis_name=x_axis_name, y_axis_name=y_axis_name, plot_name=plot_name)
    ## x-Bias
    plot_name = "acc-bias-y"
    print(plot_name)
    x_axis_name = "t[s]"
    y_axis_name = "b[m/s^2]"
    plot_2d(x_coords=self.time, y_coords=self.y_acc_bias, x_axis_name=x_axis_name, y_axis_name=y_axis_name, plot_name=plot_name)
    ## x-Bias
    plot_name = "acc-bias-z"
    print(plot_name)
    x_axis_name = "t[s]"
    y_axis_name = "b[m/s^2]"
    plot_2d(x_coords=self.time, y_coords=self.z_acc_bias, x_axis_name=x_axis_name, y_axis_name=y_axis_name, plot_name=plot_name)


  def subscriber_callback(self, state_estimate):
    print("Received state message.")
    # Time
    if not self.initial_time:
      self.initial_time = state_estimate.header.stamp.to_sec()
    self.time = self.time + [state_estimate.header.stamp.to_sec() - self.initial_time]
    # Position
    self.x_pos = self.x_pos + [state_estimate.data[0]]
    self.y_pos = self.y_pos + [state_estimate.data[1]]
    self.z_pos = self.z_pos + [state_estimate.data[2]]
    # Velocity
    self.x_vel = self.x_vel + [state_estimate.data[3]]
    self.y_vel = self.y_vel + [state_estimate.data[4]]
    # Acc. Bias
    self.x_acc_bias = self.x_acc_bias + [state_estimate.data[13]]
    self.y_acc_bias = self.y_acc_bias + [state_estimate.data[14]]
    self.z_acc_bias = self.z_acc_bias + [state_estimate.data[15]]

  def start_plotting(self):
    print("======================")
    print("Starting plotting...")
    print("======================")
    rospy.Subscriber(self.state_topic_name, DoubleArrayStamped,
                         self.subscriber_callback)

    rospy.spin()
    self.plot()

if __name__ == "__main__":
  time.sleep(1.0)
  experiment_name = input("Enter experiment name (used for file naming):")
  msf_plotter = MsfPlotter()
  msf_plotter.start_plotting()