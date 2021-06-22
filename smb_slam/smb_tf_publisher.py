#!/usr/bin/env python3  
import roslib
import rospy
import numpy
import math
import tf
import rospkg
import rosbag
from nav_msgs.msg import Odometry




odometryTopic = "/camera/odom/sample"
odometrySourceFrame = "realsense_t265"
desiredOdomFrame = "odom"
baseFrame = "base_link"
lidarFrame = "rslidar"


def computeOdomToBase(transOdomToOdomSource, rotOdomToOdomSource):
    trans1_mat = tf.transformations.translation_matrix(transSensorToBase)
    rot1_mat   = tf.transformations.quaternion_matrix(rotSensorToBase)
    T_odomSource_base = numpy.dot(trans1_mat, rot1_mat)

    trans2_mat = tf.transformations.translation_matrix(transOdomToOdomSource)
    rot2_mat    = tf.transformations.quaternion_matrix(rotOdomToOdomSource)
    T_odom_odomSource = numpy.dot(trans2_mat, rot2_mat)

    T_odom_base = numpy.dot(T_odom_odomSource, T_odomSource_base)
    trans = tf.transformations.translation_from_matrix(T_odom_base)
    rot = tf.transformations.quaternion_from_matrix(T_odom_base)

    return (trans,rot)

    

def odomCallback(msg):
    pose = msg.pose.pose
    p = pose.position
    q = pose.orientation
    br = tf.TransformBroadcaster()
    (transOdomToBase, rotOdomToBase) = computeOdomToBase((p.x, p.y, p.z),
                     (q.x,q.y,q.z,q.w))
    br.sendTransform(transOdomToBase,
                     rotOdomToBase,
                     msg.header.stamp,
                     baseFrame,
                     desiredOdomFrame)
    br.sendTransform(transBaseToLidar,
                     rotBaseToLidar,
                     msg.header.stamp,
                     lidarFrame,
                     baseFrame)



if __name__ == '__main__':
    rospy.init_node('smb_tf_pub')



    desiredOdomFrame = rospy.get_param("~odometry_frame")

    print("Launching the tf publisher node: ")
    print("Desired odom frame is %s " % desiredOdomFrame)



    listener = tf.TransformListener()

    listener.waitForTransform(odometrySourceFrame, baseFrame, rospy.Time(), rospy.Duration(60.0))
    listener.waitForTransform(lidarFrame, baseFrame, rospy.Time(), rospy.Duration(60.0))
    try:
        (transSensorToBase,rotSensorToBase) = listener.lookupTransform(odometrySourceFrame, baseFrame, rospy.Time(0))
        (transBaseToLidar,rotBaseToLidar) = listener.lookupTransform(baseFrame, lidarFrame, rospy.Time(0))
    except:
        print("exception while looking shit up")

    print("Trans cam to base: t[%f, %f, %f] \n" % (transSensorToBase[0],transSensorToBase[1],transSensorToBase[2]))
    print("Trans base to lidar: t[%f, %f, %f] \n" % (transBaseToLidar[0],transBaseToLidar[1],transBaseToLidar[2]))     

    rospy.Subscriber(odometryTopic, Odometry, odomCallback)
    rospy.spin()

        
        