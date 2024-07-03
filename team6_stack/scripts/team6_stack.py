#!/usr/bin/env python
import rospy

from std_msgs.msg import Header
from object_detection_msgs import ObjectDetectionInfoArray

# TODO: needs to be tested

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
    
    # Save the detected artifacts to a csv file
    with open('data/artifacts.csv', mode='a', newline='') as file:
        writer = csv.writer(file)
        for info in infos:
            writer.writerow([info['class_id'], info['x'], info['y'], info['z']])

def object_detection_listener():
    rospy.init_node('team6_stack', anonymous=True)
    
    # Subscribe to the /object_detector/detection_info topic
    rospy.Subscriber('/object_detector/detection_info', ObjectDetectionInfoArray, callback)
    
    # Keep the script running
    rospy.spin()

if __name__ == '__main__':
    object_detection_listener()
