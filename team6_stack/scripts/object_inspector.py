#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_msgs
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PointStamped, PoseStamped
import tf2_geometry_msgs
from object_detection_msgs.msg import ObjectDetectionInfoArray

import numpy as np

""" @TODO:
- Go to artefact with some distance from it
- Ensure that the waypoint is reachable and not in collision
- How to handle multiple artefacts?
- When to remove artefact from the list?
    - Inspect object from multiple valid angles

Process:
- Artefact is detected and position in world frame is sent through /artefact_point
- Append list if artefact is new
- If artefact list is not empty, go towards the first artefact in list with some distance
    - Distance differs with different object
    - Inspect the object from different viable angles (and distance?)
- Remove artefact from list after nice inspection, and append to inspected list
"""

class Object:
    def __init__(self, name, id, min_dist, max_dist):
        self.name = name
        self.id = id
        self.min_dist = min_dist
        self.max_dist = max_dist

EPS_ = 0.15              # m, minimum distance between two artefacts
TIMER_PERIOD_ = 1.0      # s, timer period

OBJECTS_ = {
    'backpack'  : Object('backpack', 11, 0.0, 1.0),
    'umbrella'  : Object('umbrella', 24, 0.0, 1.0),
    'bottle'    : Object('bottle', 25, 0.0, 1.0),
    'stop_sign' : Object('stop_sign', 39, 0.0, 1.0),
    'clock'     : Object('clock', 74, 0.0, 1.0),
}
MAX_NO_OBJECTS_ = 10

class ObjectInspectorNode(object):
    def __init__(self) -> None:
        rospy.init_node('my_node')
        rospy.on_shutdown(self.shutdown)

        self.inspected_artefacts = []       # list of artefacts that are already detected
        self.artefacts = []                 # list of artefacts detected in PointStamped format
        self.is_inspecting = False          #

        self.timer = rospy.Timer(rospy.Duration(0, int(TIMER_PERIOD_ * 1e9)), self.timer_callback)
        self.detected_artefacts_sub = rospy.Subscriber('/object_detector/detection_info_global', ObjectDetectionInfoArray, self.detected_artefacts_callback)
        self.inspected_artefacts_pub = rospy.Publisher('/object_inspector/detection_info_global', ObjectDetectionInfoArray, queue_size=1)
        # self.waypoint_pub = rospy.Publisher('/way_point', PointStamped, queue_size=1)

        rospy.loginfo_once('Object Inspector node is running!')
    
    def run(self) -> None:
        rospy.spin()

    def shutdown(self) -> None:
        rospy.loginfo("Shutting down...")

    def detected_artefacts_callback(self, msg: ObjectDetectionInfoArray) -> None:
        for detected_artefact in msg.info:
            already_added = False

            # Check if the artefact is already in the list, or is already inspected
            for artefact in self.artefacts + self.inspected_artefacts:
                if self.distance(artefact, msg) < EPS_:
                    already_added = True
                    break
            if already_added:
                continue

            self.artefacts.append(detected_artefact)

    def timer_callback(self, event) -> None:
        # Check if there are any artefacts
        if not self.artefacts:
            return
        
        # Publish detected artefacts
        self.publish_inspected_artefacts()
        
    def publish_inspected_artefacts(self) -> None:
        inspected_artefacts_msg = ObjectDetectionInfoArray()
        for artefact in self.artefacts:
            inspected_artefacts_msg.info.append(artefact)
            self.inspected_artefacts.append(artefact)
        self.inspected_artefacts_pub.publish(inspected_artefacts_msg)
        rospy.loginfo('Inspected artefacts published!')
        

    def distance(self, p1: PointStamped, p2: PointStamped) -> float:
        return np.sqrt(
            (p1.point.x - p2.point.x)**2 + \
            (p1.point.y - p2.point.y)**2 + \
            (p1.point.z - p2.point.z)**2
        )

def main() -> None:
    try:
        node = ObjectInspectorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()