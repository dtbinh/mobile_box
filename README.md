===========================================================
=   / _ )/ __ \| |/_/ / _ \/ / / / __/ // /  _/ |/ / ___/ =
=  / _  / /_/ />  <  / ___/ /_/ /\ \/ _  // //    / (_ /  = 
= /____/\____/_/|_| /_/   \____/___/_//_/___/_/|_/\___/   =
===========================================================                                    

The box pushing ROS Package
 - Gilberto Marcon
 - Golden Rockefeller
 - Ovunc Tuzel

Overview:
	This package includes:
		- A box domain planner service node
		- A client for solving box pushing problems in real life
		- A planning map generator
		- Custom box domain messages
		- Visualization tool in rviz

Steps for pushing boxes in the real world:
	1) A pgm map is needed for navigation. This is simply obtained by the gmapping node with a robot capable of publishing a laser scan.
	2) In order to plan in the box pushing domain, a planning map separate from the navigation map has to be generated. The map to csv python file converts an arbitrary pgm file into a grid of specified size.
	3) Instructions for box pushing in the real world:
	 	- 2 terminals have to be running on the turtlebots (ssh link). In one terminal, the turtlebot bringup code has to be running. 
	 		$ roslaunch turtlebot_bringup minimal.launch
	 	- On another terminal, the navigation code has to be running.
	 		$ roslaunch turtlebot_navigation amcl_demo.launch map_file:=yourmap.pgm
	 	- If the map topic is not published, the following command helps:
	 		$ rosrun map_server map_server yourmap.pgm
	 	- On the workspace computer, the following nodes has to be running on separate terminals:
	 		- The rviz visualization tool:
	 			$ roslaunch turtlebot_rviz_visualization view_navigation.launch
	 		- The grid visualizer for displaying the planning map on rviz (publishes to markerArray):
	 			$ rosrun box visualization_markers.py
	 		- The planning service node:
	 			$ rosrun box box_plan
	 		- The planning client node:
	 		 	$ rosrun box boxnav.py
	 	- After running the client node, the terminal will ask for an confirmation for the starting position. Give a nav goal in rviz, and press enter in the terminal. Then the terminal will ask for the starting box position. Click publish point on rviz and click on the initial box position, then press enter. Finally, click on publish point again, and click on the desired box position in rviz. Once the user hits enter, the planner will compute a path, and the robot will execute it.
