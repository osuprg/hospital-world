#!/usr/bin/env python3

# Python things
import networkx as nx
from random import random, randint, sample, choice
import numpy as np
from statistics import mean, stdev, StatisticsError
import pickle
from math import sqrt


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
        trav_data_no = nx.get_edge_attributes(self.hosp_graph, 'trav_data_no')
        trav_data_hum = nx.get_edge_attributes(self.hosp_graph, 'trav_data_hum')

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

            # try:
            if real_data:
                # print('real data')
                edge_cost = choice(trav_data_no[n1, n2] + trav_data_hum[n1, n2])
                hum_dist = 0
            else:
                if human_on_edge:
                    # print('human')
                    edge_cost = np.random.normal(means_hum[n1, n2], std_hum[n1, n2])
                    hum_dist = self.hosp_graph[n1][n2]['sq_dist']
                else:
                    # print('no human')
                    edge_cost = np.random.normal(means_no[n1, n2], std_no[n1, n2])
                    hum_dist = 0

            # except (TypeError, KeyError) as e:
            #
            #     edge_cost = self.fallback(self.hosp_graph[n1][n2])

            if edge_cost <= 0:
                edge_cost = 0.1

            self.hosp_graph[n1][n2]['temp_edge_cost'] = edge_cost
            self.hosp_graph[n1][n2]['temp_dist_with_hum'] = hum_dist

    def run_samples(self, curr_node, goal_node):
        for (n1, n2) in self.hosp_graph.edges():
            self.set_temp_graph(n1, n2)

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

    def run_backward_v2(self, paths, iterations, real_data=False):

        paths_costs = [[] for _ in range(len(paths))]
        hum_dist_all = [[] for _ in range(len(paths))]
        # paths_costs = []
        for _ in range(iterations):

            self.set_temp_graph(real_data)

            for j in range(len(paths)):
                [_, path] = paths[j]
                hum_dist = 0
                # print(path)
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

    def run_backward(self, path, iterations, ignore_hum=False, real_data=False):
        '''
        This takes a path as an input and returns the average and standard deviation of the cost of that path over
        a given number of iterations
        '''

        path_costs = []

        for _ in range(iterations):
            path_len = 0

            for i in range(len(path) - 1):
                n1 = path[i]
                n2 = path[i + 1]
                self.set_temp_graph(n1, n2, real_data, ignore_hum)
                path_len += self.hosp_graph[n1][n2]['temp_edge_cost']

            path_costs.append(path_len)

        return mean(path_costs), stdev(path_costs), min(path_costs), max(path_costs)

    def gaussian_combo(self, paths, hum_cond='all'):
        low = 0
        high = 0
        path_mean = 0
        var = 0
        pct_hum = nx.get_edge_attributes(self.hosp_graph, 'pct_hum')
        means_hum = nx.get_edge_attributes(self.hosp_graph, 'mean_hum')
        std_hum = nx.get_edge_attributes(self.hosp_graph, 'std_hum')
        means_no = nx.get_edge_attributes(self.hosp_graph, 'mean_no')
        std_no = nx.get_edge_attributes(self.hosp_graph, 'std_no')
        mean_all = nx.get_edge_attributes(self.hosp_graph, 'mean_all')
        std_all = nx.get_edge_attributes(self.hosp_graph, 'std_all')
        hum_interval = nx.get_edge_attributes(self.hosp_graph, 'weight_hum')
        no_interval = nx.get_edge_attributes(self.hosp_graph, 'weight_no')

        [_, path] = paths

        for i in range(len(path) - 1):
            n1 = path[i]
            n2 = path[i + 1]
            try:
                curr_pct_hum = pct_hum[n1, n2]
            except KeyError:
                dummy = n1
                n1 = n2
                n2 = dummy
                curr_pct_hum = pct_hum[n1, n2]
            # print(n1, n2, means_hum[n1, n2])
            if hum_cond == "no" or means_hum[n1, n2] is None:
                # print('no', hum_cond)
                path_mean += means_no[n1, n2]
                low += no_interval[n1, n2].low
                high += no_interval[n1, n2].high
                var += std_no[n1, n2] ** 2
            elif hum_cond == 'hum':
                # print('hum', hum_cond)
                path_mean += means_hum[n1, n2]
                low += hum_interval[n1, n2].low
                high += hum_interval[n1, n2].high
                var += std_hum[n1, n2] ** 2
            else:
                # print('all', hum_cond)
                path_mean += mean_all[n1, n2]
                low += min([hum_interval[n1, n2].low, no_interval[n1, n2].low])
                high += max([hum_interval[n1, n2].high, no_interval[n1, n2].high])
                var += std_all[n1, n2] ** 2

        std = sqrt(var)
        return path_mean, std, low, high


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

