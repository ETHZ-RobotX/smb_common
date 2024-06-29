#!/usr/bin/env python3

# Global
import matplotlib.pyplot as plt
import numpy as np
import os
import rospy
import sys
import time

# Custom
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped

# Name of the experiment
experiment_name = ""


def plot_2d(x_coords: list, y_coords: list, x_axis_name: str, y_axis_name: str, plot_name: str, log_dir: str):
    # Check length
    print("Length of " + x_axis_name + ": " + str(len(x_coords)) + ". Length of " + y_axis_name + ": " + str(
        len(y_coords)))
    # Only plot if there are values
    if len(x_coords) > 0:
        assert np.abs(len(x_coords) - len(y_coords)) <= 1
        if len(x_coords) - len(y_coords) == 1:
            x_coords.pop()  # remove last element
        elif len(y_coords) - len(x_coords) == 1:
            y_coords.pop()

        # Plot
        plt.rcParams['lines.markersize'] = 0.5
        plt.rcParams["figure.figsize"] = (6, 6)
        plt.xticks(fontsize=24)
        plt.yticks(fontsize=24)
        fig, ax = plt.subplots(1, 1)
        ax.set(xlabel=x_axis_name, ylabel=y_axis_name, title=plot_name)
        ax.grid()
        plt.tight_layout()
        ax.plot(x_coords, y_coords, c="green", linewidth=0.7, alpha=1.0)
        fig.savefig(os.path.join(log_dir, experiment_name + "_" + plot_name + ".png"))


class MsfPlotter:

    def __init__(self):

        self.state_odometry_topic_name = "/graph_msf/est_odometry_world_imu"
        self.wheel_odometry_topic_name = "/control/smb_diff_drive/odom"
        self.imu_bias_topic_name = "/graph_msf/accel_bias"

        # ROS
        rospy.init_node('GMSF Plotter Node', anonymous=True)

        # Create dir
        HOME_DIR = os.path.expanduser('~')
        if not os.path.exists(os.path.join(HOME_DIR, "rss_plots")):
            os.makedirs(os.path.join(HOME_DIR, "rss_plots"))

        # State Messages
        ## Time
        self.initial_state_time = []
        self.state_time = []
        ## Position
        self.x_pos = []
        self.y_pos = []
        self.z_pos = []
        ## Velocity
        self.x_vel = []
        self.y_vel = []
        # IMU Bias Messages
        ## Time
        self.initial_imu_bias_time = []
        self.imu_bias_time = []
        ## Acc Bias
        self.x_acc_bias = []
        self.y_acc_bias = []
        self.z_acc_bias = []
        # Wheel odometry position
        ## Time
        self.initial_wheel_time = []
        ## Position
        self.x_wheel_pos = []
        self.y_wheel_pos = []
        self.z_wheel_pos = []

    def plot(self, log_dir: str):

        # Plot diagrams when destructor is called
        print(f"Saving plots to {log_dir}...")

        # Position plots -------------------------------------------
        if experiment_name == "1.1" or experiment_name == "1.2" or experiment_name == "1.3":
            ## x-y-plot
            plot_name = "state-position-x-y"
            print(plot_name)
            x_axis_name = "x[m]"
            y_axis_name = "y[m]"
            plot_2d(x_coords=self.x_pos, y_coords=self.y_pos, x_axis_name=x_axis_name, y_axis_name=y_axis_name,
                    plot_name=plot_name, log_dir=log_dir)

        if experiment_name == "2.1" or experiment_name == "2.2" or experiment_name == "3.1" or experiment_name == "3.2":
            ## z-plot
            plot_name = "state-position-z"
            print(plot_name)
            x_axis_name = "t[s]"
            y_axis_name = "z[m]"
            plot_2d(x_coords=self.state_time, y_coords=self.z_pos, x_axis_name=x_axis_name, y_axis_name=y_axis_name,
                    plot_name=plot_name, log_dir=log_dir)

        # Velcoity plots -------------------------------------------
        if experiment_name == "1.1" or experiment_name == "1.2" or experiment_name == "1.3" or experiment_name == "3.1" or experiment_name == "3.2":
            ## x-velocity
            plot_name = "state-velocity-x"
            print(plot_name)
            x_axis_name = "t[s]"
            y_axis_name = "v[m/s]"
            plot_2d(x_coords=self.state_time, y_coords=self.x_vel, x_axis_name=x_axis_name, y_axis_name=y_axis_name,
                    plot_name=plot_name, log_dir=log_dir)
            ## y-velocity
            plot_name = "state-velocity-y"
            print(plot_name)
            x_axis_name = "t[s]"
            y_axis_name = "v[m/s]"
            plot_2d(x_coords=self.state_time, y_coords=self.y_vel, x_axis_name=x_axis_name, y_axis_name=y_axis_name,
                    plot_name=plot_name, log_dir=log_dir)

        # Bias plots -----------------------------------------------
        if experiment_name == "2.1" or experiment_name == "2.2":
            ## x-Bias
            plot_name = "acc-bias-x"
            print(plot_name)
            x_axis_name = "t[s]"
            y_axis_name = "b[m/s^2]"
            plot_2d(x_coords=self.imu_bias_time, y_coords=self.x_acc_bias, x_axis_name=x_axis_name,
                    y_axis_name=y_axis_name,
                    plot_name=plot_name, log_dir=log_dir)
            ## y-Bias
            plot_name = "acc-bias-y"
            print(plot_name)
            x_axis_name = "t[s]"
            y_axis_name = "b[m/s^2]"
            plot_2d(x_coords=self.imu_bias_time, y_coords=self.y_acc_bias, x_axis_name=x_axis_name,
                    y_axis_name=y_axis_name,
                    plot_name=plot_name, log_dir=log_dir)
            ## z-Bias
            plot_name = "acc-bias-z"
            print(plot_name)
            x_axis_name = "t[s]"
            y_axis_name = "b[m/s^2]"
            plot_2d(x_coords=self.imu_bias_time, y_coords=self.z_acc_bias, x_axis_name=x_axis_name,
                    y_axis_name=y_axis_name,
                    plot_name=plot_name, log_dir=log_dir)

        # Wheel odometry plots -------------------------------------
        if experiment_name == "1.2":
            ## x-y-plot
            plot_name = "wheel-position-x-y"
            print(plot_name)
            x_axis_name = "x[m]"
            y_axis_name = "y[m]"
            plot_2d(x_coords=self.x_wheel_pos, y_coords=self.y_wheel_pos, x_axis_name=x_axis_name,
                    y_axis_name=y_axis_name,
                    plot_name=plot_name, log_dir=log_dir)

    def state_odometry_callback(self, state_estimate):
        # Time
        if not self.initial_state_time:
            self.initial_state_time = state_estimate.header.stamp.to_sec()
            print("Received state message.")
            return
        elif state_estimate.header.stamp.to_sec() - self.initial_state_time > 0.5:
            self.state_time = self.state_time + [state_estimate.header.stamp.to_sec() - self.initial_state_time]
            # Position
            self.x_pos = self.x_pos + [state_estimate.pose.pose.position.x]
            self.y_pos = self.y_pos + [state_estimate.pose.pose.position.y]
            self.z_pos = self.z_pos + [state_estimate.pose.pose.position.z]
            # Velocity
            self.x_vel = self.x_vel + [state_estimate.twist.twist.linear.x]
            self.y_vel = self.y_vel + [state_estimate.twist.twist.linear.y]

    def wheel_odometry_callback(self, wheel_odometry):
        # Time
        if not self.initial_wheel_time:
            self.initial_wheel_time = wheel_odometry.header.stamp.to_sec()
            print("Received wheel message.")
            return
        elif wheel_odometry.header.stamp.to_sec() - self.initial_wheel_time > 0.5:
            self.x_wheel_pos = self.x_wheel_pos + [wheel_odometry.pose.pose.position.x]
            self.y_wheel_pos = self.y_wheel_pos + [wheel_odometry.pose.pose.position.y]
            self.z_wheel_pos = self.z_wheel_pos + [wheel_odometry.pose.pose.position.z]

    def bias_callback(self, bias_estimate):
        # Time
        if not self.initial_imu_bias_time:
            self.initial_imu_bias_time = bias_estimate.header.stamp.to_sec()
            print("Received bias message.")
            return
        elif bias_estimate.header.stamp.to_sec() - self.initial_imu_bias_time > 0.5:
            self.imu_bias_time = self.imu_bias_time + [bias_estimate.header.stamp.to_sec() - self.initial_imu_bias_time]
            # Acc. Bias
            self.x_acc_bias = self.x_acc_bias + [bias_estimate.vector.x]
            self.y_acc_bias = self.y_acc_bias + [bias_estimate.vector.y]
            self.z_acc_bias = self.z_acc_bias + [bias_estimate.vector.z]

    def start_plotting(self, log_dir: str):
        print("======================")
        print("Starting plotting...")
        print("======================")

        # Odometry Subscriber
        rospy.Subscriber(self.state_odometry_topic_name, Odometry,
                         self.state_odometry_callback)

        # Wheel Odometry Subscriber
        rospy.Subscriber(self.wheel_odometry_topic_name, Odometry,
                         self.wheel_odometry_callback)

        # Bias Subscriber
        rospy.Subscriber(self.imu_bias_topic_name, Vector3Stamped,
                         self.bias_callback)

        # Start spinning
        rospy.spin()

        # Plot after shutdown
        self.plot(log_dir=log_dir)


if __name__ == "__main__":
    time.sleep(1.0)
    if len(sys.argv) > 1:
        log_dir = sys.argv[1]
        print("Log directory (where plots will be saved to): " + log_dir)
    else:
        raise ValueError("Please provide the log directory.")
    # Input
    experiment_name = input("Enter experiment name (used for file naming):")
    # Run Plotter
    msf_plotter = MsfPlotter()
    msf_plotter.start_plotting(log_dir=log_dir)
