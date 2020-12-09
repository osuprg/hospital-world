# hospital_world

## Basics:

The purpose of this package is to provide a custom global planner that can account for both the dynamics of the environment and the urgency of the task at hand. 

The robot will rely on data collected over time (a.k.a. long-term learning) about how long it takes to traverse a specific area.
This accounts for the reality of the environment, including dynamics (e.g. humans walking around), unforeseen obstacles, or challenging areas (areas where the robot frequently fails). 
When the robot plans a path, it will rely on the learned traversal times to plan its route, rather than relying solely on distance. 

In the current implementation, dynamics are represented by 'human presence'; the world determines probabilistically whether or not humans are present on a given edge. 
If humans are present, it slows the robot down proportionally to how many humans are 'detected'. 
Currently there are three possible presence conditions, with different numbers of humans present and probabilities of how often they are around.

Our method is broken into two main stages, with many overlapping methods and functions. 
These are separate stages because we want to wait to collect data until after the initial learning stage.
* **STAGE 1**: Collect long-term learning (LTL) data
* **STAGE 2**: Run custom global planner on robot using LTL data


## Dependencies:

#### ROS Packages:
* Turtlebot3-*
* Navstack
* dwa-local-planner

#### Python packages:
* Networkx

## STAGE 1: Collect LTL data

#### Terminal 1 
```
export TURTLEBOT3_MODEL=burger
roslaunch hospital-world turtlebot_hospital.launch
```

#### Terminal 2
```
rosrun hospital-world RUN_trials.py
```

#### Terminal 3 
```
rosrun hospital-world RUN_publish_node_location.py 
```

#### Terminal 4
```
rosrun hospital-world RUN_gather_edge_data.py
```


## STAGE 2: Use LTL data in global planner

#### Terminal 1 
```
export TURTLEBOT3_MODEL=burger
roslaunch hospital-world turtlebot_hospital.launch
```

#### Terminal 2
```
rosrun hospital-world RUN_global_planner.py
```

#### Terminal 3 
```
rosrun hospital-world RUN_publish_node_location.py 
```



## Components in this package:


### Structural elements
File names start with "STRUCT_"
* **hospital_graph_class.py** - creates a NetworkX graph of the hospital, including nodes and edges.
* **hospital_graph.py** - example of how to use hospital_graph_class.py - matches the Hospital_world_graph_sketch.jpg (with minor edits)
* **interval_cust.py** - Custom interval class. Used to store data about edges in the graph. 
* **hospital_v1_parameters_raw.txt** - parameters file that outlines the general structure and x,y locations of each node / edge
* **plot_nodes.py** - a good check for setting up a new world. 
  Allows you to plot the NetworkX graph in 2D space to ensure everything is located and linked correctly.

### Run trials
File names start with "RUN_"
* **trials.py** - moves the robot in the specified world. Chooses a node, navigates to the node, then chooses another, etc. for a specified number of iterations. 
* **publish_node_location.py** - has two main functions. These are written in the same node for synchronization:
    * Publishes which node the robot is currently in - updates as often as /amcl_pose - outputs node to topic 
    * Determines the human condition on that specific edge. 
      Decides whether or not humans are present for this iteration based on the human condition. 
      Changes the top speed of the robot depending on presence / absence of humans and publishes the current condition.
* **gather_edge_data.py** - 
  subscribes to topic outputting the robot's current node - times how long it takes to switch nodes, updates the 'weight' interval (in hospital class). 
  Pickles and saves file every 5 minutes and on exit. Make sure to check / change the file path where you want it to save - specified just after imports.

### Analyze data
File names start with "POST_"
* **add_stats_to_edges.py** - this takes the data collected then runs basic statistics and saves for each edge (e.g. mean and variance of each distribution)
* **analyze_data.py** - plots the distribution of times collected on each edge


## Still to do:

Structural:
* 
* Write launch files (because 5 terminal launch sequence is ridiculous)
    * Gather 'long-term learning' data
    * Run custom global planner using LTL data
    * Run custom global planner vs dijkstra and collect data

* Make the parameters file integrate with the hospital graph - no reason to store that info in two places
* What to do about rotate recovery? I guess leave it in because it might be a navigation error challenge - what if it oscillates between two nodes really fast?


Done: 
-
* ~~Redo the timing logic - nodes are currently large spaces, so 'reaching' the node could be anywhere from one end of the hall to the other - messes up the timing.~~
* ~~Print interval out to file (pickle the hospital world when exiting)~~
* ~~Make this into a ros package instead of a bunch of random files~~
* ~~Clean up move_base file - some of the comments / commented out sections are leftover from an old project~~
