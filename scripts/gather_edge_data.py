#!/usr/bin/env python

import rospy

from hospital_graph_class import HospitalGraph
from std_msgs.msg import String
from time import time
from two_rooms_v3_parameters import TwoRoomsParameters as p


class RobotTiming:
    def __init__(self, param):
        self.p = param
        self.current_node = None
        self.prior_node = None
        self.start_time = None
        self.end_time = None
        self.building = HospitalGraph(p.num_rooms, p.num_halls, p.extra_doors,
                                      p.hall_door_links, p.extra_door_hall_links, p.connected_halls)

    def set_current_node(self, msg):
        self.prior_node = self.current_node
        self.current_node = msg.data

        if self.prior_node != self.current_node and self.prior_node:
            self.end_time = time()
            self.get_timing()
        if not self.start_time:
            self.start_time = time()

    def get_timing(self):
        traversal_time = self.end_time - self.start_time
        self.start_time = self.end_time
        self.record_timing(traversal_time)

    def record_timing(self, trav_time):
        # Purely for simplicity of reading
        print(self.current_node)
        print(self.prior_node)
        low = self.building.G[self.current_node][self.prior_node]['weight'].low
        high = self.building.G[self.current_node][self.prior_node]['weight'].high
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
    node_sub = rospy.Subscriber('node_in_graph', String, r.set_current_node)
    rospy.spin()
