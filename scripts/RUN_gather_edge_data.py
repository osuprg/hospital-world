#!/usr/bin/env python3

import rospy

from std_msgs.msg import String
from time import time
import atexit
from datetime import date

import STRUCT_hospital_graph_class as HospGraph

# Yes this is terribly dumb, but you have to write out the full filepath
# Otherwise it will save to the directory from which you are running the script
path_and_name = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/hospital_trials_{}'.format(date.today())


class RobotTiming:
    def __init__(self, hosp_graph):
        # Keeps track of current and prior node to check when it changes
        self.current_node = None
        self.next_node = None

        # Start and end time for current edge
        self.start_time = None
        self.end_time = None

        self.hum_cond = None

        self.start_pickle_timer = time()
        # Set up a NetworkX graph of the building / hospital
        self.hosp_graph = hosp_graph

    def set_current_node(self, msg):
        # Logic for the first loop
        if not self.start_time:
            self.start_time = time()

        # Otherwise record the timing data
        else:
            self.end_time = time()

            # if the prior message had node information, record info from prior edge
            if self.current_node:
                self.record_timing()
            # Otherwise, reset the timer
            else:
                self.start_time = time()

        # AFTER recording data from prior edge, update info from new message
        # If the current message does not contain node information, set all info to None
        if msg.data == "Do not record":
            self.current_node = None
            self.next_node = None
            self.hum_cond = None
        # Otherwise record edge info
        else:
            [self.current_node, self.next_node, self.hum_cond] = msg.data.split(', ')

        # Saves file every 5 minutes or so
        if time() - self.start_pickle_timer >= 300:
            HospGraph.pickle_it(self.hosp_graph, path_and_name)
            self.start_pickle_timer = time()

    def record_timing(self):
        # Calculate how much time it took to traverse that edge
        trav_time = self.end_time - self.start_time

        # Start time of the new edge is the end time of the prior edge
        self.start_time = self.end_time

        try:
            # Get the current interval bounds
            # Purely for simplicity of reading
            low = self.hosp_graph[self.current_node][self.next_node]['weight' + self.hum_cond].low
            high = self.hosp_graph[self.current_node][self.next_node]['weight' + self.hum_cond].high

            # Print stuff to make sure this is working correctly
            # print("Moved from {0} to {1}".format(self.next_node, self.current_node))
            # print('previous interval: [{0}, {1}]'.format(low, high))
            # print('traversal time', trav_time)

            # If the travel time is less than the current low value or greater than the highest
            # Also check if low or high == 0, the default starting value
            if low > trav_time or low == 0:
                self.hosp_graph[self.current_node][self.next_node]['weight' + self.hum_cond].low = trav_time
            if high < trav_time or high == 0:
                self.hosp_graph[self.current_node][self.next_node]['weight' + self.hum_cond].high = trav_time
            # print('new interval: ', self.hosp_graph[self.current_node][self.next_node]['weight'])
            self.hosp_graph[self.current_node][self.next_node]['trav_data' + self.hum_cond].append(trav_time)
            print(self.next_node, self.current_node, self.hum_cond, trav_time)
            print("traversal data set", self.hosp_graph[self.current_node][self.next_node]['trav_data' + self.hum_cond])
        except KeyError:
            print("those nodes are not connected")


if __name__ == "__main__":
    rospy.init_node('gather_edge_data_py')
    path_to_pickle = rospy.get_param('graph_file_path')

    hosp_graph = HospGraph.unpickle_it(path_to_pickle)

    r = RobotTiming(hosp_graph)

    # Subscribe to the topic that publishes which node the robot is in
    node_sub = rospy.Subscriber('node_in_graph', String, r.set_current_node)

    # When the script exits, it will pickle and save the file
    atexit.register(HospGraph.pickle_it, obj_to_pickle=r.hosp_graph, file_path_and_name=path_and_name)

    rospy.spin()

