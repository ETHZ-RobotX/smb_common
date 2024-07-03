#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_msgs
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs

from tf2_msgs.msg import TFMessage
from custom_msgs.msg import DetectionInfo  # Adjust this import based on the actual message type
import numpy as np

tf_buffer = None
tf_listener = None

def tf_static_callback(msg):
    for transform in msg.transforms:
        # base_link -> rgb_camera_link
        if transform.header.frame_id == "base_link" and transform.child_frame_id == "rgb_camera_link":
            translation_base2cam = transform.transform.translation
            rotation_base2cam = transform.transform.rotation
            rospy.loginfo(f"Translation_base2cam: x={translation_base2cam.x}, y={translation_base2cam.y}, z={translation_base2cam.z}")
            rospy.loginfo(f"Rotation_base2cam: x={rotation_base2cam.x}, y={rotation_base2cam.y}, z={rotation_base2cam.z}, w={rotation_base2cam.w}")
        # rgb_camera_link -> rgb_camera_optical_link
        if transform.header.frame_id == "rgb_camera_link" and transform.child_frame_id == "rgb_camera_optical_link":
            translation_cam2opt = transform.transform.translation
            rotation_cam2opt = transform.transform.rotation
            rospy.loginfo(f"Translation_cam2opt: x={translation_cam2opt.x}, y={translation_cam2opt.y}, z={translation_cam2opt.z}")
            rospy.loginfo(f"Rotation_cam2opt: x={rotation_cam2opt.x}, y={rotation_cam2opt.y}, z={rotation_cam2opt.z}, w={rotation_cam2opt.w}")

def tf_callback(msg):
    for transform in msg.transforms:
        if transform.header.frame_id == "base_link" and transform.child_frame_id == "rgb_camera_link":
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            #rospy.loginfo(f"Translation: x={translation.x}, y={translation.y}, z={translation.z}")
            #rospy.loginfo(f"Rotation: x={rotation.x}, y={rotation.y}, z={rotation.z}, w={rotation.w}")

def detection_callback(msg):
    for detection in msg.info:
        if detection.frame_id == "rgb_camera_optical_link":
            object_position = detection.position
            class_id = detection.class_id

            # Create a PoseStamped message for the detected object's position
            object_pose = tf2_geometry_msgs.PoseStamped()
            object_pose.header.frame_id = "rgb_camera_optical_link"
            object_pose.pose.position = object_position
            object_pose.pose.orientation.w = 1.0

            try:
                # Transform the object position to the base_link frame
                transformed_pose = tf_buffer.transform(object_pose, "base_link", rospy.Duration(1.0))

                transformed_position = transformed_pose.pose.position
                rospy.loginfo(f"Detected object (class_id: {class_id}) position in base_link frame: x={transformed_position.x}, y={transformed_position.y}, z={transformed_position.z}")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr(f"TF transform error: {e}")

def duplicate_rejection():
    #TODO: if the global postions of multiple detected objects are too close. Keep one and reject others. 

def main():
    global tf_buffer, tf_listener

    rospy.init_node('tf_listener', anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    rospy.Subscriber('/tf_static', tf2_msgs.msg.TFMessage, tf_static_callback)
    rospy.Subscriber('/tf_static', TFMessage, tf_callback)
    rospy.Subscriber('/object_detector/detection_info', DetectionInfo, detection_callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()
