#!/usr/bin/env python

import csv 
import sys
import rospy, roslib
import sys, actionlib, random
from geometry_msgs.msg import Point, PoseStamped, PointStamped
import rospkg
from box.msg import Map
from box.msg import Problem
from box.msg import Plan
from nav_msgs.msg import Odometry
from box.srv import *
from box_plan_client import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
import math
boxpos_publisher = rospy.Publisher('boxpos', Point, queue_size=10)
goalpos_publisher = rospy.Publisher('goalpos', Point, queue_size=10)

startRovers = []
startBoxes = []
endBoxes = []
boxStartSet = False
boxGoalSet = False

NAVMAPX = 8
NAVMAPY = 6
PLANMAPX = 20
PLANMAPY = 17
OFFSETX = 0.05
OFFSETY = 0.0

def updateCurrentPos(pos):
	global startRovers
	startRovers = [pos.pose.position]
	# print "Current position updated!"


def updateBoxPos(pos):
	global startBoxes, endBoxes, boxGoalSet, boxStartSet
	if not boxStartSet:
		startBoxes = [pos.point]
		# print "Box start position updated!"
		boxStartSet = True
	elif not boxGoalSet:
		endBoxes = [pos.point]
		# print "Box goal position updated!"
		# print "Ready to go!"
		boxGoalSet = True

def send_goal(x, y, th=0):
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0
    # quaternion = quaternion_from_euler(0, 0, th)
    # goal.target_pose.pose.orientation.x = quaternion[0]
    # goal.target_pose.pose.orientation.y = quaternion[1]
    # goal.target_pose.pose.orientation.z = quaternion[2]
    # goal.target_pose.pose.orientation.w = quaternion[3]
    goal.target_pose.pose.orientation.x = globalOdom.pose.pose.orientation.x
    goal.target_pose.pose.orientation.y = globalOdom.pose.pose.orientation.y
    goal.target_pose.pose.orientation.z = globalOdom.pose.pose.orientation.z
    goal.target_pose.pose.orientation.w = globalOdom.pose.pose.orientation.w
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5))

def send_angle_goal(prev_xy, next_xy):
    goal.target_pose.pose.position.x = prev_xy[0]
    goal.target_pose.pose.position.y = prev_xy[1]
    goal.target_pose.pose.position.z = 0
    quaternion = quaternion_from_euler(0, 0, math.atan2(next_xy[1] - prev_xy[1], next_xy[0] - prev_xy[0]))
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5))

    goal.target_pose.pose.position.x = next_xy[0]
    goal.target_pose.pose.position.y = next_xy[1]
    goal.target_pose.pose.position.z = 0
    quaternion = quaternion_from_euler(0, 0, math.atan2(next_xy[1] - prev_xy[1], next_xy[0] - prev_xy[0]))    
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5))

def solve_problem(roverStart, boxStart, boxEnd):
	# File parameters
	rospack = rospkg.RosPack()
	map_filename = rospack.get_path('box')+"/maps/PlanningMap.csv"
	map = load_map(map_filename)

	problem = Problem()
	problem.num_rovers = len(roverStart)
	problem.num_boxes = len(boxStart)

	for pt in roverStart:
		problem.initial_rover.append(pt)
	for pt in boxStart:
		problem.initial_box.append(pt)
	for pt in boxEnd:
		problem.final_box.append(pt)

	plan = request_plan(map,problem)
	print_plan(plan)
	return plan

def planner2worldPts(pointList):
	for pt in pointList:
		yield pt.y*NAVMAPX/PLANMAPX + OFFSETX, -pt.x*NAVMAPY/PLANMAPY + OFFSETY

def world2plannerPts(pointList):
    for pt in pointList:
        yield Point(-int(round((pt.y - OFFSETY)/NAVMAPY*PLANMAPY)), int(round((pt.x - OFFSETX)/NAVMAPX*PLANMAPX)), pt.z)

def returnPlan(roverStart, boxStart, boxEnd):
	roverStart = list(world2plannerPts(roverStart))
	boxStart = list(world2plannerPts(boxStart))
	boxEnd = list(world2plannerPts(boxEnd))
	plan = solve_problem(roverStart, boxStart, boxEnd)
	return plan



if __name__ == "__main__":	
	rospy.init_node('boxnav')
	# startRovers = [Point(7.38,-4.8,0)]
	# startBoxes = [Point(6.7,-2.5,0)]
	# endBoxes = [Point(5.5,-2.32, 0)]

	sub1 = rospy.Subscriber('move_base_simple/goal', PoseStamped, updateCurrentPos)
	sub2 = rospy.Subscriber('clicked_point', PointStamped, updateBoxPos)
	raw_input('Please move the robot to start position ... ')
	raw_input('Please click on the box start position ... ')
	raw_input('Please click on the box goal position ... ')

	goalpos_publisher.publish(Point(list(planner2worldPts(list(world2plannerPts(endBoxes))))[0][0], list(planner2worldPts(list(world2plannerPts(endBoxes))))[0][1], 0))
	plan = returnPlan(startRovers, startBoxes, endBoxes)
	waypoints = list(planner2worldPts(plan.rover_pos))
	# print 'Ready to go!'
	# print waypoints


	

	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()

	flag_goHome = 0
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"

	print "GO!:"
	for g in range(len(waypoints)):
		pt = list(planner2worldPts(plan.box_pos))[g]
		boxpos_publisher.publish(Point(pt[0],pt[1],0))
		if g > 0:
			send_angle_goal(previous_goal, (waypoints[g][0], waypoints[g][1]))
		else:
			send_angle_goal((0,0), (waypoints[g][0], waypoints[g][1]))
		# send_goal(waypoints[g][0], waypoints[g][1])
		previous_goal = (waypoints[g][0], waypoints[g][1])
		print "Ok"