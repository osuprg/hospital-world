#!/usr/bin/env python3

# ROS things
# import rospy

# Python things
from time import time
import networkx as nx
from random import random, randint, sample
import numpy as np
from statistics import mean, stdev, StatisticsError

# Custom things
# CHANGE THESE TO POINT TO YOUR PARAMETERS FILE INFO
import STRUCT_hospital_graph_class as HospGraph
# from std_msgs.msg import String


class SamplingPlannerClass:
    def __init__(self, hosp_graph):
        self.paths_dict = {}
        self.hosp_graph = hosp_graph
        self.current_key = 0

    def get_path(self, samples, n1, n2):
        for _ in range(samples):
            self.run_samples(n1, n2)

        key = self.analyze_results()
        new_path = self.paths_dict[key][0]
        return new_path

    def run_samples(self, curr_node, goal_node):
        pct_hum = nx.get_edge_attributes(self.hosp_graph, 'pct_hum')
        means_hum = nx.get_edge_attributes(self.hosp_graph, 'mean_hum')
        std_hum = nx.get_edge_attributes(self.hosp_graph, 'std_hum')
        means_no = nx.get_edge_attributes(self.hosp_graph, 'mean_no')
        std_no = nx.get_edge_attributes(self.hosp_graph, 'std_no')

        start = time()

        # Each time we run a sample, we're going to
        for (n1, n2) in self.hosp_graph.edges():
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

                self.hosp_graph[n1][n2]['temp_edge_cost'] = edge_cost
                # print(n1, n2, '{:0.3f}'.format(edge_cost))

            except (TypeError, KeyError) as e:
                # print('Key error for edge [{}, {}]. Human condition {}'.format(n1, n2, human_on_edge))
                # There is no data for this edge, so we will set the cost to a very high value
                self.hosp_graph[n1][n2]['temp_edge_cost'] = 1000

        new_length, new_path = nx.single_source_dijkstra(self.hosp_graph, curr_node, target=goal_node, weight='temp_edge_cost')
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

    def re_init(self):
        self.paths_dict = {}
        self.current_key = 0


if __name__ == "__main__":

    # Use this if running on robot / with ROS up
    # This takes 0.16407 seconds on startup - significantly slower but it only has to happen once per node so... meh?
    # rospy.init_node('sampling_global_planner')
    # path_to_pickle = rospy.get_param('graph_file_path')

    # Use this for testing
    # This takes 0.00048 seconds
    path_to_pickle = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/hospital_trials_2020-12-19_plus_stats'

    hosp_graph = HospGraph.unpickle_it(path_to_pickle)
    planner = SamplingPlannerClass(hosp_graph)

    total = 0
    rooms = [i for i in range(21)]
    same = 0
    different_count = 0
    iterations = 10000
    sample_size = 1000

    # Looking at room pairings that would potentially have multiple paths.
    # Makes no sense to compare methods if there is only one realistic path in any situation
    room_pairs = [[8, 9, 10, 12, 13, 14, 15],  # 0
                  [7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17],  # 1
                  [11, 12, 13, 14, 15, 16, 17, 18],  # 2
                  [11, 12, 13, 14, 15, 16, 17, 18],  # 3
                  [11, 12, 13, 14, 15, 16, 17, 18],  # 4
                  [11, 12, 13, 14, 15, 16, 17, 18],  # 5
                  [17, 18, 19, 20],  # 6
                  [17, 18, 19, 20],  # 7
                  [0, 1, 19, 20],  # 8
                  [0, 1, 19, 20],  # 9
                  [0, 1, 19, 20],  # 10
                  [0, 1, 2, 3, 4, 5, 19, 20],  # 11
                  [0, 1, 2, 3, 4, 5, 19, 20],  # 12
                  [0, 1, 2, 3, 4, 5, 19, 20],  # 13
                  [0, 1, 2, 3, 4, 5, 19, 20],  # 14
                  [0, 1, 2, 3, 4, 5, 19, 20],  # 15
                  [0, 1, 2, 3, 4, 5, 19, 20],  # 16
                  [0, 1, 2, 3, 4, 5, 6, 7],  # 17
                  [0, 1, 2, 3, 4, 5, 6, 7],  # 18
                  [6, 7, 8, 9, 10, 11, 12, 13, 14, 15],  # 19
                  [6, 7, 8, 9, 10, 11, 12, 13, 14, 15]]  # 20



    for num in range(iterations):
        # Pick a random start and end room
        rm1 = randint(0, 20)
        rm2 = sample(room_pairs[rm1], 1)[0]  # This comes out as an array with one item, so we just call that item
        n1 = 'r' + "{:02d}".format(rm1)
        n2 = 'r' + "{:02d}".format(rm2)

        start_time = time()
        planner = SamplingPlannerClass(hosp_graph)
        custom = planner.get_path(sample_size, n1, n2)
        dijkstra = nx.dijkstra_path(hosp_graph, n1, n2, weight='euclidean_dist')

        diff = [i for i in dijkstra if i not in custom]
        diff2 = [j for j in custom if j not in dijkstra]

        if len(diff) > 0 and len(diff2) > 0:

            print('----------- {} -------------'.format(num))
            print('path from {} to {}'.format(n1, n2))
            print('cust: {}'.format(custom))
            print('dijk: {}'.format(dijkstra))
            different_count += 1
            print(diff, diff2)

        #     # print(time() - start_time)
        total += time() - start_time

    print("average time: {}".format(total / iterations))
    print("percent of same path: {}".format((1 - (different_count / iterations)) * 100))

