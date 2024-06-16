#!/usr/bin/env python3

import rospy
import rosparam
import subprocess
import time

class EKFParameterMonitor:
    def __init__(self):
        rospy.init_node('ekf_parameter_monitor')

        self.namespace = "/ekf_se"
        self.params_to_monitor = [
            "odom_frame", "base_link_frame", "world_frame",
            "two_d_mode", "frequency", "publish_tf", "print_diagnostics",
            "odom0", "odom0_config", "odom0_queue_size", "odom0_differential", "odom0_relative",
            "imu0", "imu0_config", "imu0_differential", "imu0_queue_size", "imu0_remove_gravitational_acceleration",
            "odom1", "odom1_config", "odom1_queue_size", "odom1_differential", "odom1_relative",
            "diagnostic_updater/frequency", "diagnostic_updater/min_frequency", "diagnostic_updater/max_frequency"
        ]

        self.previous_params = self.get_current_params()
        self.check_interval = rospy.Duration(1.0)  # Check every second

        self.timer = rospy.Timer(self.check_interval, self.check_params)

    def get_current_params(self):
        current_params = {}
        for param in self.params_to_monitor:
            full_param = self.namespace + "/" + param
            if rospy.has_param(full_param):
                current_params[param] = rospy.get_param(full_param)
        return current_params

    def check_params(self, event):
        current_params = self.get_current_params()

        if current_params != self.previous_params:
            rospy.loginfo("Parameters changed, restarting EKF node.")
            self.restart_ekf_node()
            self.previous_params = current_params

    def restart_ekf_node(self):
        # Shutdown the EKF node
        subprocess.call(["rosnode", "kill", "/ekf_se"])

        # Allow some time for the node to shut down
        time.sleep(1)

        # Relaunch the EKF node
        subprocess.Popen(["roslaunch", "smb_control", "start_ekf_reconfigure.launch"])

        # Wait for the node to initialize
        time.sleep(2)

if __name__ == "__main__":
    try:
        monitor = EKFParameterMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
