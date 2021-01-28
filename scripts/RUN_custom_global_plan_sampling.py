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
import RUN_custom_global_plan_algebraic as Alg



class SamplingPlannerClass:
    def __init__(self, hosp_graph):
        self.paths_dict = {}
        self.hosp_graph = hosp_graph
        self.current_key = 0

    def get_path(self, samples, n1, n2):
        for _ in range(samples):
            self.run_samples(n1, n2)

        average, key = self.analyze_results()
        new_path = self.paths_dict[key][0]
        return average, new_path

    def run_samples(self, curr_node, goal_node):

        # Get edge attributes. This is not necessary, but makes the code easier to read
        pct_hum = nx.get_edge_attributes(self.hosp_graph, 'pct_hum')
        means_hum = nx.get_edge_attributes(self.hosp_graph, 'mean_hum')
        std_hum = nx.get_edge_attributes(self.hosp_graph, 'std_hum')
        means_no = nx.get_edge_attributes(self.hosp_graph, 'mean_no')
        std_no = nx.get_edge_attributes(self.hosp_graph, 'std_no')

        # Each time we run a sample, we're going to
        for (n1, n2) in self.hosp_graph.edges():
            # print('----------- {} to {} -----------'.format(n1, n2))
            # Decide whether or not there will be a human on the edge
            human_on_edge = False

            percent_on_edge = pct_hum[n1, n2]
            if percent_on_edge and random() < percent_on_edge:
                human_on_edge = True

            # print('human: ', human_on_edge)

            # Then get the timing for that edge
            try:
                if human_on_edge:
                    edge_cost = np.random.normal(means_hum[n1, n2], std_hum[n1, n2])
                    # print('HUM | mean: {} | std: {}'.format(means_hum[n1, n2], std_hum[n1, n2]))
                else:
                    edge_cost = np.random.normal(means_no[n1, n2], std_no[n1, n2])
                    # print('NO | mean: {} | std: {}'.format(means_no[n1, n2], std_no[n1, n2]))

                if edge_cost <= 0:
                    edge_cost = 0.1

                self.hosp_graph[n1][n2]['temp_edge_cost'] = edge_cost
                # print(n1, n2, '{:0.3f}'.format(edge_cost))

            except (TypeError, KeyError) as e:
                # print('Key error for edge [{}, {}]. Human condition {}'.format(n1, n2, human_on_edge))
                # Divide the euclidean distance by the top speed of the robot, dampened by 30%
                if human_on_edge:
                    self.hosp_graph[n1][n2]['temp_edge_cost'] = self.hosp_graph[n1][n2]['euclidean_dist'] / (0.22 * 0.5 * 0.8)
                else:
                    self.hosp_graph[n1][n2]['temp_edge_cost'] = self.hosp_graph[n1][n2]['euclidean_dist'] / (0.22 * 0.8)

                # print('exception reached, {} - {}'.format(n1, n2))

            # print("{} - {} : {} : {}".format(n1, n2, human_on_edge, self.hosp_graph[n1][n2]['temp_edge_cost']))

        new_length, new_path = nx.single_source_dijkstra(self.hosp_graph, curr_node, target=goal_node, weight='temp_edge_cost')
        # print(new_path)
        added = False

        for key, [path, values] in self.paths_dict.items():
            diff1 = [i for i in new_path if i not in path]
            diff2 = [j for j in path if j not in new_path]
            if len(diff1) == 0 or len(diff2) == 0:
                self.paths_dict[key][1].append(new_length)
                added = True
                break

        if not added:
            self.paths_dict['key{}'.format(self.current_key)] = [new_path, [new_length]]
            self.current_key += 1

    def analyze_results(self):
        # Hold the mean, standard deviation, quartile, num chosen, and key of the path with the smallest mean
        min_mean = [10000, 10000, 10000, 0, None]
        min_quartile = [10000, 10000, 10000, 0, None]
        max_chosen = [10000, 10000, 10000, 0, None]

        for key, [path, values] in self.paths_dict.items():
            # Only use paths that have been sampled at least 5 times
            average = mean(values)
            try:
                std = stdev(values)
            except StatisticsError:
                std = 0

            upper_quartile = average + 2 * std
            num_chosen = len(values)
            self.paths_dict[key].append([mean, std, upper_quartile, num_chosen])
            if upper_quartile < min_mean[2]:
                min_quartile = [average, std, upper_quartile, num_chosen, key]

            if average < min_mean[0]:
                min_mean = [average, std, upper_quartile, num_chosen, key]

            if num_chosen > max_chosen[3]:
                max_chosen = [average, std, upper_quartile, num_chosen, key]

            # print(path, values)
            # print(average, std)

        # if min_quartile[0] != min_mean[0]:
        #     print('quartile: {}'.format(min_quartile[:3]))
        #     print('mean: {}'.format(min_mean[:3]))

        return max_chosen[0], max_chosen[4]

    def re_init(self):
        self.paths_dict = {}
        self.current_key = 0


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

