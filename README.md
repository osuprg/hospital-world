# hospital_world

Contains the following:

Structural elements:
* hospital_graph_class.py - creates a NetworkX graph of the hospital, including nodes and edges.
* hospital_graph.py - example of how to use hospital_graph_class.py - matches the Hospital_world_graph_sketch.jpg (with minor edits)
* interval_cust.py - Sets 

Two rooms v3 world:
* Basic world set up for testing purposes - world and map files in worlds folder
* two_rooms_v3_parameters.py - parameters file that outlines the general structure and x,y locations of each node / edge

Scripts that do stuff:
* move_base.py - Moves the robot around in the world - currently set up to move back and forth between two rooms
* publish_node_location.py - publishes which node the robot is currently in - updates as often as /amcl_pose - outputs node to topic 
* gather_edge_data.py - subscribes to topic outputting the robot's current node - times how long it takes to switch nodes, updates the 'weight' interval (in hospital class). Currently just prints to screen but will be updated to read from / print to file


Dependencies:
-
ROS Packages:
* Turtlebot3-*
* Navstack
* dwa-local-planner

Python packages:
* Networkx

 To Run:
-
* Terminal 1 
```
export TURTLEBOT3_MODEL=burger
roslaunch hospital_world turtlebot_two_rooms.launch
```
* Terminal 2
```
rosrun hospital_world move_base.py
```
* Terminal 3 
```
rosrun hospital_world publish_node_location.py 
```
* Terminal 4 
```
rosrun hospital_world gather_edge_data.py
```

Still to do:
- 
* Launch all nodes from one launch file
* Redo the timing logic - nodes are currently large spaces, so 'reaching' the node could be anywhere from one end of the hall to the other - messes up the timing.
* ~~Print interval out to file (pickle the hospital world when exiting)~~
* ~~Make this into a ros package instead of a bunch of random files~~
* ~~Clean up move_base file - some of the comments / commented out sections are leftover from an old project~~
