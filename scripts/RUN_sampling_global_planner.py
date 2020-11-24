#!/usr/bin/env python3

# ROS things
# import rospy

# Python things
from time import time
import networkx as nx
from random import random
import numpy as np
from statistics import mean, stdev, StatisticsError

# Custom things
# CHANGE THESE TO POINT TO YOUR PARAMETERS FILE INFO
import STRUCT_hospital_graph_class as HospGraph


class SamplingPlannerClass:
    def __init__(self, hosp_graph):
        self.paths_dict = {}
        self.hosp_graph = hosp_graph
        self.current_key = 0

    def get_path(self, samples):
        for _ in range(samples):
            self.run_samples()

        key = self.analyze_results()
        return self.paths_dict[key][0]

    def run_samples(self):
        pct_hum = nx.get_edge_attributes(hosp_graph, 'pct_hum')
        means_hum = nx.get_edge_attributes(hosp_graph, 'mean_hum')
        std_hum = nx.get_edge_attributes(hosp_graph, 'std_hum')
        means_no = nx.get_edge_attributes(hosp_graph, 'mean_no')
        std_no = nx.get_edge_attributes(hosp_graph, 'std_no')

        start = time()

        # TODO: Once done testing, put this in a loop to run a lot of samples
        # Each time we run a sample, we're going to
        for (n1, n2) in hosp_graph.edges():
            # Decide whether or not there will be a human on the edge, then get timing
            human_on_edge = False
            percent_on_edge = pct_hum[n1, n2]
            if percent_on_edge and random() < percent_on_edge:
                human_on_edge = True

            try:
                if human_on_edge:
                    edge_cost = np.random.normal(means_hum[n1, n2], std_hum[n1, n2])

                else:
                    edge_cost = np.random.normal(means_no[n1, n2], std_no[n1, n2])

                if edge_cost <= 0:
                    edge_cost = 0.1

                hosp_graph[n1][n2]['temp_edge_cost'] = edge_cost
                # print(n1, n2, '{:0.3f}'.format(edge_cost))

            except (TypeError, KeyError) as e:
                # print('Key error for edge [{}, {}]. Human condition {}'.format(n1, n2, human_on_edge))
                # There is no data for this edge, so we will set the cost to a very high value
                hosp_graph[n1][n2]['temp_edge_cost'] = 1000

        new_length, new_path = nx.single_source_dijkstra(hosp_graph, 'r16', target='r04', weight='temp_edge_cost')
        added = False
        for key, [path, values] in self.paths_dict.items():
            if new_path == path:
                self.paths_dict[key][1].append(new_length)
                added = True
                break

        if not added:
            self.paths_dict['key{}'.format(self.current_key)] = [new_path, [new_length]]
            self.current_key += 1

    def analyze_results(self):
        # Hold the mean, standard deviation, and key of the path with the smallest mean
        min_mean = [10000, 10000, None]

        for key, [path, values] in self.paths_dict.items():
            # Only use paths that have been sampled at least 5 times
            if len(values) > 5:
                average = mean(values)
                std = stdev(values)
                self.paths_dict[key].append([mean, std])
                if average < min_mean[0]:
                    min_mean = [average, std, key]

            # Otherwise we don't have enough information about this path
            else:
                average = 1000
                std = 1000

        return min_mean[2]


if __name__ == "__main__":

    # Use this if running on robot / with ROS up
    # This takes 0.16407 seconds on startup - significantly slower but it only has to happen once per node so... meh?
    # rospy.init_node('sampling_global_planner')
    # path_to_pickle = rospy.get_param('graph_file_path')

    # Use this for testing
    # This takes 0.00048 seconds
    path_to_pickle = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/hospital_trials_2020-10-23_plus_stats'

    hosp_graph = HospGraph.unpickle_it(path_to_pickle)

    total = 0
    # for i in range(100):
    #     start_time = time()
    #
    for _ in range(100):
        planner = SamplingPlannerClass(hosp_graph)
        print(planner.get_path(1000))

    #     # print(time() - start_time)
    #     total += time() - start_time
    #
    # print(total / 100)
