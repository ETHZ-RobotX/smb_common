#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_msgs
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, PointStamped
import tf2_geometry_msgs
from object_detection.object_detection_msgs import ObjectDetectionInfoArray
from my_transform_package.msg import TransformedDetection

import numpy as np

tf_buffer = None
tf_listener = None

translation_odomGraph2worldGraph = None
rotation_odomGraph2worldGraph = None
translation_base2odomGraph = None
rotation_base2odomGraph = None
translation_world2base = None
rotation_world2base = None
translation_base2cam = None
rotation_base2cam = None
translation_cam2opt = None
rotation_cam2opt = None

transformed_detection_pub = None

def tf_callback(msg):
        
    global translation_odomGraph2worldGraph, rotation_odomGraph2worldGraph
    global translation_base2odomGraph, rotation_base2odomGraph
    global translation_world2base, rotation_world2base

    for transform in msg.transforms:
        # world -> base_link
        if transform.header.frame_id == "odom_graph_msf" and transform.child_frame_id == "world_graph_msf"
            translation_odomGraph2worldGraph = transform.transform.translation
            rotation_odomGraph2worldGraph = transform.transform.rotation
        if transfrom.header.frame_id == "base_link" and transform.child_frame_id == "odom_graph_msf":
            translation_base2odomGraph = transform.transform.translation
            rotation_base2odomGraph = transform.transform.rotation

        if transform.header.frame_id == "odom" and transform.child_frame_id == "base_link":
            translation_world2base = transform.transform.translation
            rotation_world2base = transform.transform.rotation
            rospy.loginfo(f"Translation_world2base: x={translation_world2base.x}, y={translation_world2base.y}, z={translation_world2base.z}")
            rospy.loginfo(f"Rotation_world2base: x={rotation_world2base.x}, y={rotation_world2base.y}, z={rotation_world2base.z}, w={rotation_world2base.w}")


def tf_static_callback(msg):
    global translation_base2cam, rotation_base2cam
    global translation_cam2opt, rotation_cam2opt

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


def detection_callback(msg):
    for detection in msg.info:
        if detection.frame_id == "rgb_camera_optical_link":
            object_position = detection.position
            class_id = detection.class_id

            # Create a PoseStamped message for the detected object's position
            object_pose = tf2_geometry_msgs.PoseStamped()
            object_pose.header.frame_id = "rgb_camera_optical_link"
            object_pose.pose.position = object_position
            object_pose.pose.orientation.w = 1.0 # Don't care about the orientation

            try:
                # Transform the object position to the base_link frame
                transformed_pose = tf_buffer.transform(object_pose, "base_link", rospy.Duration(1.0))
                transformed_position = transformed_pose.pose.position
                rospy.loginfo(f"Detected object (class_id: {class_id}) position in base_link frame: x={transformed_position.x}, y={transformed_position.y}, z={transformed_position.z}")
            
                # Publish the transformed detection
                transformed_detection = TransformedDetection()
                transformed_detection.position = transformed_position
                transformed_detection.class_id = class_id
                transformed_detection_pub.publish(transformed_detection)
            
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr(f"TF transform error: {e}")

def duplicate_rejection(detections):
    unique_detections = []
    for i, detection in enumerate(detections):
        is_duplicate = False
        for unique_detection in unique_detections:
            dist = np.sqrt(
                (detection.position.x - unique_detection.position.x) ** 2 +
                (detection.position.y - unique_detection.position.y) ** 2 +
                (detection.position.z - unique_detection.position.z) ** 2
            )
            if dist < 0.1:  # If objects are closer than 10 cm, consider them duplicates
                is_duplicate = True
                break
        if not is_duplicate:
            unique_detections.append(detection)
    return unique_detections

def main():
    global tf_buffer, tf_listener, transformed_detection_pub

    rospy.init_node('tf_listener', anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    rospy.Subscriber('/tf', TFMessage, tf_callback)
    rospy.Subscriber('/tf_static', TFMessage, tf_static_callback)
    rospy.Subscriber('/object_detector/detection_info', ObjectDetectionInfoArray, detection_callback)
    
    transformed_detection_pub = rospy.Publisher('/transformed_detections', TransformedDetection, queue_size=10)
    #rospy.Publisher('/artefact_point', PointStamped)
    rospy.spin()

if __name__ == '__main__':
    main()
