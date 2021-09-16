#!/usr/bin/env python3

# Python things
import networkx as nx
from random import random, choice
import numpy as np
import pickle


class SamplingPlannerClass:
    def __init__(self, hosp_graph):
        self.paths_dict = {}
        self.hosp_graph = hosp_graph
        self.current_key = 0

    def fallback(self, dict):
        euclid = dict['euclidean_dist']
        # return euclid / (0.22 * 0.98)
        return 1000

    def set_temp_graph(self, real_data=False, ignore_hum=False):
        # Get edge attributes. This is not necessary, but makes the code easier to read
        pct_hum = nx.get_edge_attributes(self.hosp_graph, 'pct_hum')
        means_hum = nx.get_edge_attributes(self.hosp_graph, 'mean_hum')
        std_hum = nx.get_edge_attributes(self.hosp_graph, 'std_hum')
        means_no = nx.get_edge_attributes(self.hosp_graph, 'mean_no')
        std_no = nx.get_edge_attributes(self.hosp_graph, 'std_no')

        for (n1, n2) in self.hosp_graph.edges():
            human_on_edge = False
            try:
                curr_pct_hum = pct_hum[n1, n2]
            except KeyError:
                dummy = n1
                n1 = n2
                n2 = dummy
                curr_pct_hum = pct_hum[n1, n2]

            if not ignore_hum:
                # Decide whether or not there will be a human on the edge
                if curr_pct_hum and random() < curr_pct_hum:
                    human_on_edge = True

            if human_on_edge:
                edge_cost = np.random.normal(means_hum[n1, n2], std_hum[n1, n2])
                hum_dist = self.hosp_graph[n1][n2]['sq_dist']
            else:
                # print('no human')
                edge_cost = np.random.normal(means_no[n1, n2], std_no[n1, n2])
                hum_dist = 0

            if edge_cost <= 0:
                edge_cost = 0.1

            self.hosp_graph[n1][n2]['temp_edge_cost'] = edge_cost
            self.hosp_graph[n1][n2]['temp_dist_with_hum'] = hum_dist

    def re_init(self):
        self.paths_dict = {}
        self.current_key = 0

    def run_backward_v2(self, paths, iterations):

        paths_costs = [[] for _ in range(len(paths))]
        hum_dist_all = [[] for _ in range(len(paths))]
        # paths_costs = []
        for _ in range(iterations):

            self.set_temp_graph()

            for j in range(len(paths)):
                path = paths[j]
                hum_dist = 0
                path_len = 0
                for i in range(len(path) - 1):
                    n1 = path[i]
                    n2 = path[i + 1]
                    path_len += self.hosp_graph[n1][n2]['temp_edge_cost']
                    hum_dist += self.hosp_graph[n1][n2]['temp_dist_with_hum']

                paths_costs[j].append(path_len)
                hum_dist_all[j].append(hum_dist)
                # paths_costs.append(path_len)
        return paths_costs, hum_dist_all


class DataAccumulated:
    def __init__(self):
        self.paths = []
        self.times = []


def unpickle_it(filename):
    infile = open(filename, 'rb')
    unpickled = pickle.load(infile)
    infile.close()
    return unpickled


def pickle_it(obj_to_pickle, file_path_and_name):
    with open(file_path_and_name, 'wb') as f:
        pickle.dump(obj_to_pickle, f)
    print("Saving information to file named {}".format(file_path_and_name))


if __name__ == "__main__":
    pass
    # Use this if running on robot / with ROS up
    # This takes 0.16407 seconds on startup - significantly slower but it only has to happen once per node so... meh?
    # rospy.init_node('sampling_global_planner')
    # path_to_pickle = rospy.get_param('graph_file_path')

