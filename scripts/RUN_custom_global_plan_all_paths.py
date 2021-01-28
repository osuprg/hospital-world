#!/usr/bin/env python3

# ROS things
# import rospy

# Python things
from time import time
import networkx as nx
from random import random, randint, sample
import numpy as np
from statistics import mean, stdev, StatisticsError
import pickle

# Custom things
import STRUCT_hospital_graph_class as HospGraph


class AllPathsPlanner:
    def __init__(self, hosp_graph):
        self.paths_dict = {}
        self.hosp_graph = hosp_graph

    def all_paths(self, rm1, rm2):
        means = []
        vars = []

        # Get edge attributes. This is not necessary, but makes the code easier to read
        pct_hum = nx.get_edge_attributes(self.hosp_graph, 'pct_hum')
        means_hum = nx.get_edge_attributes(self.hosp_graph, 'mean_hum')
        std_hum = nx.get_edge_attributes(self.hosp_graph, 'std_hum')
        means_no = nx.get_edge_attributes(self.hosp_graph, 'mean_no')
        std_no = nx.get_edge_attributes(self.hosp_graph, 'std_no')
        euclid = nx.get_edge_attributes(self.hosp_graph, 'euclidean_dist')
        all_paths = []

        for path in nx.all_simple_paths(self.hosp_graph, source=rm1, target=rm2):
            all_paths.append(path)
            path_mean = 0
            path_var = 0
            for i in range(len(path) - 1):
                n1 = path[i]
                n2 = path[i + 1]
                try:
                    dummy = std_no[n1, n2]
                except KeyError:
                    n0 = n1
                    n1 = n2
                    n2 = n0

                try:
                    path_mean += (1 - pct_hum[n1, n2]) * means_no[n1, n2] + pct_hum[n1, n2] * means_hum[n1, n2]
                    path_var += (1 - pct_hum[n1, n2]) * std_no[n1, n2] + pct_hum[n1, n2] * std_hum[n1, n2]
                except:
                    try:
                        path_mean += means_no[n1, n2]
                        path_var += std_no[n1, n2]
                    except:
                        path_mean += euclid[n1, n2] / 0.22
                        path_var += 6.0

            means.append(path_mean)
            vars.append(path_var)

        min_val = min(means)
        min_index = np.argmin(means)
        return all_paths[min_index]


if __name__ == "__main__":
    path_and_name = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/hospital_trials_2021-01-15_plus_stats_clean'

    hosp_graph = HospGraph.unpickle_it(path_and_name)
    planner = AllPathsPlanner(hosp_graph)
    planner.all_paths()


