#!/usr/bin/env python

import rospy

import hospital_graph_class
from std_msgs.msg import String
from time import time
import atexit
from datetime import date

# CHANGE THIS TO POINT TO YOUR PARAMETERS FILE
from two_rooms_v3_parameters import TwoRoomsParameters as p


class RobotTiming:
    def __init__(self, param):
        # Parameters
        self.p = param

        # Keeps track of current and prior node to check when it changes
        self.current_node = None
        self.prior_node = None

        # Start and end time for current edge
        self.start_time = None
        self.end_time = None

        # Set up a NetworkX graph of the building / hospital
        self.building = hospital_graph_class.HospitalGraph(p.num_rooms, p.num_halls, p.extra_doors,
                                      p.hall_door_links, p.extra_door_hall_links, p.connected_halls)

    def set_current_node(self, msg):
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

    def record_timing(self, trav_time):
        # Calculate how much time it took to traverse that edge
        trav_time = self.end_time - self.start_time

        # Start time of the new edge is the end time of the prior edge
        self.start_time = self.end_time

        # Get the current interval bounds
        # Purely for simplicity of reading
        low = self.building.G[self.current_node][self.prior_node]['weight'].low
        high = self.building.G[self.current_node][self.prior_node]['weight'].high

        # Print stuff to make sure this is working correctly
        print("Moved from {0} to {1}".format(self.prior_node, self.current_node))
        print('previous interval: [{0}, {1}]'.format(low, high))
        print('traversal time', trav_time)

        # If the travel time is less than the current low value or greater than the highest
        # Also check if low or high == 0, the default starting value
        if low > trav_time or low == 0:
            self.building.G[self.current_node][self.prior_node]['weight'].low = trav_time
        if high < trav_time or high == 0:
            self.building.G[self.current_node][self.prior_node]['weight'].high = trav_time
        print('new interval: ', self.building.G[self.current_node][self.prior_node]['weight'])


if __name__ == "__main__":
    rospy.init_node('gather_edge_data_py')
    r = RobotTiming(p)

    # Subscribe to the topic that publishes which node the robot is in
    node_sub = rospy.Subscriber('node_in_graph', String, r.set_current_node)

    # Yes this is terribly dumb, but you have to write out the full filepath
    # Otherwise it will save to the directory from which you are running the script
    path_and_name = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/hospital_interval_{}'.format(date.today())

    # When the script exits, it will pickle and save the file
    atexit.register(hospital_graph_class.pickle_it, obj_to_pickle=r.building, file_path_and_name=path_and_name)

    rospy.spin()

