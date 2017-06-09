#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math

topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray, queue_size=10)
waypoints = [(20.96, 4.06), (21.15, 7.25), (20, 7.33), (19.84, 5.73), (17.42, 5.43), (17.52, 4.17)]
rospy.init_node('markers')

markerArray = MarkerArray()
while not rospy.is_shutdown():
    # print len(markerArray.markers)
    for i in range(len(waypoints)):
        if len(markerArray.markers) < len(waypoints):
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = waypoints[i][0]
            marker.pose.position.y = waypoints[i][1]
            markerArray.markers.append(marker)
    #
    # if (count > MARKERS_MAX):
    #     markerArray.markers.pop(0)
    id = 0
    for m in markerArray.markers:
        m.id = id
        id += 1

    # Publish the MarkerArray
    publisher.publish(markerArray)
    rospy.sleep(0.01)
