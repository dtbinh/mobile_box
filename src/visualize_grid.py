#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import rospy, rospkg
import math, csv
global boxMarker, goalMarker
boxMarker = (0,0)
goalMarker = (-1,-1)
NAVMAPX = 8.0
NAVMAPY = 5.75
PLANMAPX = 20
PLANMAPY = 15
OFFSETX = 0.55
OFFSETY = 0.1

topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray, queue_size=10)
# waypoints = [(20.96, 4.06), (21.15, 7.25), (20, 7.33), (19.84, 5.73), (17.42, 5.43), (17.52, 4.17)]
rospy.init_node('markers')

def drawBox(pt):
    print pt
    global boxMarker
    print 'Box Moved!'
    boxMarker = (pt.x, pt.y)

def drawGoals(pt):    
    print pt
    global goalMarker
    goalMarker = (pt.x, pt.y)

def readCsv(path):
    points = []
    with open(path, 'rb') as csvfile:
        map = csv.reader(csvfile, delimiter=',')
        map = list(map)
    for row in range(len(map)):
        for col in range(len(map[row])):
            if map[row][col] == '0':
                points.append((float(col), float(row), 1))
            else:
                points.append((float(col), float(row), 0))
    return points
         
def planner2worldPts(pointList):
    for xy in pointList:
        yield xy[0]*NAVMAPX/PLANMAPX + OFFSETX, -xy[1]*NAVMAPY/PLANMAPY - OFFSETY, xy[2]

markerArray = MarkerArray()
waypoints = readCsv(rospkg.RosPack().get_path('box')+"/maps/PlanningMap.csv")
waypoints = list(planner2worldPts(waypoints))

# for i in waypoints:
#     print i, -int(round((i[1] - OFFSETY)/NAVMAPY*PLANMAPY)), int(round((i[0] - OFFSETX)/NAVMAPX*PLANMAPX))
# Subscribe to scan


subscriber = rospy.Subscriber('boxpos', Point, drawBox)
subscriber = rospy.Subscriber('goalpos', Point, drawGoals)
while not rospy.is_shutdown():
    markerArray.markers = []
    for i in range(len(waypoints)):
        if len(markerArray.markers) < len(waypoints):
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.01
            marker.pose.orientation.w = 1.0
            marker.lifetime = rospy.Time(10)
            
            if waypoints[i][2] == 1:
                marker.color.r = 0.5
                marker.color.g = 0.7
                marker.color.b = 0.2
                marker.color.a = 0.75
            else:
                marker.color.r = 0.6
                marker.color.g = 0.1
                marker.color.b = 0.1
                marker.color.a = 1

            marker.pose.position.x = waypoints[i][0]
            marker.pose.position.y = waypoints[i][1]
            markerArray.markers.append(marker)

    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.0
    marker.scale.x = 0.25
    marker.scale.y = 0.25
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.pose.orientation.w = 1.0
    print goalMarker
    marker.pose.position.x = goalMarker[0]
    marker.pose.position.y = goalMarker[1]
    markerArray.markers.append(marker)

    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.color.r = 0.1
    marker.color.g = 0.1
    marker.color.b = 1.0
    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.pose.orientation.w = 1.0
    print boxMarker
    marker.pose.position.x = boxMarker[0]
    marker.pose.position.y = boxMarker[1]
    markerArray.markers.append(marker)

    # print len(markerArray.markers)
    #
    # if (count > MARKERS_MAX):
    #     markerArray.markers.pop(0)
    id = 0
    for m in markerArray.markers:
        m.id = id
        id += 1

    # Publish the MarkerArray
    publisher.publish(markerArray)
    rospy.sleep(0.1)
