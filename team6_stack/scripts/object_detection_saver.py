#!/usr/bin/env python
import rospy

import os
import csv
import time

from std_msgs.msg import Header
from object_detection_msgs.msg import ObjectDetectionInfoArray

timestr = ''

def callback(msg):
    infos = []
    for info in msg.info:
        info = {
            'class_id': info.class_id,
            # TODO: Add 'id' if it's not always 0
            'x': info.position.x, # in camera frame
            'y': info.position.y, # in camera frame
            'z': info.position.z # in camera frame
        }
        infos.append(info)
    
    print(infos)
    
    # Check if the CSV file exists
    dirname = os.path.dirname(__file__)
    #timestr = time.strftime("%Y%m%d-%H%M%S")
    file_name = 'artifacts-' + timestr + '.csv' # Add the already set timestamp
    #file_name = 'artifacts.csv'
    file_path = os.path.join(dirname, './../data/' + file_name) # Relative path
    file_exists = os.path.isfile(file_path)
    
    # Save the detected artifacts to a CSV file
    if file_exists:
        with open(file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            for info in infos:
                writer.writerow([info['class_id'], info['x'], info['y'], info['z']])
    
def object_detection_saver():
    rospy.init_node('object_detection_saver', anonymous=True)

    global timestr
    
    # Check if the CSV file exists
    dirname = os.path.dirname(__file__)
    timestr = time.strftime("%Y%m%d-%H%M%S")
    #file_name = 'artifacts.csv'
    file_name = 'artifacts-' + timestr + '.csv' # Add timestamp for the beginning of the run
    file_path = os.path.join(dirname, './../data/' + file_name) # Relative path
    file_exists = os.path.isfile(file_path)

    # Write the header only if the file doesn't exist
    if not file_exists:
        with open(file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['class_id', 'x', 'y', 'z'])

    # Subscribe to topics
    #rospy.Subscriber('/object_detector/detection_info', ObjectDetectionInfoArray, callback)
    rospy.Subscriber('/object_inspector/unique_artifacts', ObjectDetectionInfoArray, callback)
    
    # Keep the script running
    rospy.spin()

if __name__ == '__main__':
    object_detection_saver()
