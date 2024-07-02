#!/usr/bin/env python
import rospy

from std_msgs.msg import Header
from object_detection.object_detection_msgs import ObjectDetection

def callback(msg):
    infos = []
    for info in msg.info:
        info = {
            'class_id': info.class_id,
            'x': info.position.x, # in camera frame
            'y': info.position.y, # in camera frame
            'z': info.position.z # in camera frame
        }
        infos.append(info)
    print(infos)
    # TODO: Save to a csv file

def object_detection_listener():
    rospy.init_node('team6_stack', anonymous=True)
    
    # Subscribe to the /object_detection/detection_info topic
    rospy.Subscriber('/object_detection/detection_info', ObjectDetection, callback)
    
    # Keep the script running
    rospy.spin()

if __name__ == '__main__':
    object_detection_listener()
