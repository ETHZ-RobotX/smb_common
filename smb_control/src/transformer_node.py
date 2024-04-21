#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped

def callback(data):
    twist_msg = TwistWithCovarianceStamped()
    twist_msg.header = data.header
    twist_msg.twist = data.twist

    pub.publish(twist_msg)

if __name__ == '__main__':
    rospy.init_node('odom_transformer_node', anonymous=True)

    rospy.Subscriber('/tracking_camera/odom/sample', Odometry, callback)
    pub = rospy.Publisher('/tracking_camera/odom/sample_transformed', TwistWithCovarianceStamped, queue_size=10)

    rospy.spin()
