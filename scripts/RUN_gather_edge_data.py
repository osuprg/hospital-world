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
        self.prior_node = None

        # Start and end time for current edge
        self.start_time = None
        self.end_time = None

        self.hum_state = None

        # Set up a NetworkX graph of the building / hospital
        self.hosp_graph = hosp_graph

    def set_current_node(self, msg):
        '''
        I'm fairly confident in the timing of this - the node should update first, then the human state.
        When the node updates, it will use the human state from the prior node for the edge it just traversed
        This logic should hold up. I couldn't think of a less 'hope for the best' way to do this
        '''
        
        # Human states start with _
        if msg.data[0] == '_':
            self.hum_state = msg.data

        # Otherwise it is a node update
        else:
            # Update current and prior nodes
            self.prior_node = self.current_node
            self.current_node = msg.data

            # Checks to see if the node changed and that this is not the first loop
            if self.prior_node != self.current_node and self.prior_node:
                # If it changed nodes, mark the end time and get the timing
                self.end_time = time()
                self.record_timing()

            # Logic for the first loop
            if not self.start_time:
                self.start_time = time()

    def record_timing(self):
        # Calculate how much time it took to traverse that edge
        trav_time = self.end_time - self.start_time

        # Start time of the new edge is the end time of the prior edge
        self.start_time = self.end_time

        # Get the current interval bounds
        # Purely for simplicity of reading
        low = self.hosp_graph[self.current_node][self.prior_node]['weight' + self.hum_state].low
        high = self.hosp_graph[self.current_node][self.prior_node]['weight' + self.hum_state].high

        # Print stuff to make sure this is working correctly
        # print("Moved from {0} to {1}".format(self.prior_node, self.current_node))
        # print('previous interval: [{0}, {1}]'.format(low, high))
        # print('traversal time', trav_time)

        # If the travel time is less than the current low value or greater than the highest
        # Also check if low or high == 0, the default starting value
        if low > trav_time or low == 0:
            self.hosp_graph[self.current_node][self.prior_node]['weight' + self.hum_state].low = trav_time
        if high < trav_time or high == 0:
            self.hosp_graph[self.current_node][self.prior_node]['weight' + self.hum_state].high = trav_time
        # print('new interval: ', self.hosp_graph[self.current_node][self.prior_node]['weight'])
        self.hosp_graph[self.current_node][self.prior_node]['trav_data' + self.hum_state].append(trav_time)
        print(self.prior_node, self.current_node, self.hum_state)
        print("traversal data set", self.hosp_graph[self.current_node][self.prior_node]['trav_data' + self.hum_state])


if __name__ == "__main__":
    rospy.init_node('gather_edge_data_py')
    path_to_pickle = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/STRUCT_hospital_v1_param_pickle'

    hosp_graph = HospGraph.unpickle_it(path_to_pickle)

    r = RobotTiming(hosp_graph)

    # Subscribe to the topic that publishes which node the robot is in
    node_sub = rospy.Subscriber('node_in_graph', String, r.set_current_node)

    # TODO: do this more often (every 10 minutes?)
    # When the script exits, it will pickle and save the file
    atexit.register(HospGraph.pickle_it, obj_to_pickle=r.hosp_graph, file_path_and_name=path_and_name)

    rospy.spin()

