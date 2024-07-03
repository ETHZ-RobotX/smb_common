#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_msgs
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PointStamped, PoseStamped
import tf2_geometry_msgs

import numpy as np

eps = 1.0               # m, minimum distance between two artefacts
min_dist = 2.0          # m, minimum distance from artefact to robot
timer_period = 1.0      # s, timer period

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
- Remove artefact from list after some moment
"""

class Object:
    def __init__(self, name, id, min_dist, max_dist):
        self.name = name
        self.id = id
        self.min_dist = min_dist
        self.max_dist = max_dist

class ObjectInspectionNode(object):
    def __init__(self) -> None:
        rospy.init_node('my_node')
        rospy.on_shutdown(self.shutdown)

        self.objects = [
            Object('backpack', 11, 0.0, 1.0),
            Object('umbrella', 24, 0.0, 1.0),
            Object('bottle', 25, 0.0, 1.0),
            Object('stop_sign', 39, 0.0, 1.0),
            Object('clock', 74, 0.0, 1.0),
        ]

        self.inspected_artefacts = [] # list of artefacts that are already detected
        self.artefacts = [] # list of artefacts detected in PointStamped format

        self.timer = rospy.Timer(rospy.Duration(0, int(timer_period*1e9)), self.timer_callback)
        self.artefact_sub = rospy.Subscriber('/artefact_point', PointStamped, self.artefact_point_callback)
        self.waypoint_pub = rospy.Publisher('/way_point', PointStamped, queue_size=1)
        # self.inspection_pub = rospy.Publisher('/something something')

    def run(self) -> None:
        rospy.spin()

    def shutdown(self) -> None:
        rospy.loginfo("Shutting down...")

    def artefact_point_callback(self, msg: PointStamped) -> None:
        # Check if the artefact is already in the list
        for artefact in self.artefacts:
            if self.distance(artefact, msg) < eps:
                rospy.loginfo_throttle(30, "Duplicate artefact detected")
                return
        self.artefacts.append(msg)
        rospy.loginfo(f"Artefact detected: x={msg.point.x}, y={msg.point.y}, z={msg.point.z}")

    def timer_callback(self, event) -> None:
        # Check if there are any artefacts
        if not self.artefacts:
            return

        # Get the current robot position
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        try:
            transform = tf_buffer.lookup_transform('base_link', 'world_msf_graph', rospy.Time())
            robot_position = transform.transform.translation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to get robot position")
            return
        
        # Get the pose from the artefact to the robot in the world_msf_graph frame
        artefact = self.artefacts[0]
        pose = PoseStamped()
        pose.pose.position = artefact.point
        pose.header.frame_id = 'world_msf_graph'
        pose.header.stamp = rospy.Time()
        pose = tf_buffer.transform(pose, 'base_link')
        artefact.point = pose.pose.position



        # Check if the robot is close enough to the artefact
        if self.distance(robot_position, artefact) < min_dist:
            rospy.loginfo("Robot is inspecting the artefact")
        else:
            # Publish the artefact as a waypoint
            self.waypoint_pub.publish(artefact)

    def distance(self, p1: PointStamped, p2: PointStamped) -> float:
        return np.sqrt(
            (p1.point.x - p2.point.x)**2 + \
            (p1.point.y - p2.point.y)**2 + \
            (p1.point.z - p2.point.z)**2
        )

def main() -> None:
    try:
        node = ObjectInspectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()