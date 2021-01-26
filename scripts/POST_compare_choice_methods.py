#!/usr/bin/env python3

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
# import RUN_custom_global_plan_sampling as Samp


def pickle_it(obj_to_pickle, file_path_and_name):
    with open(file_path_and_name, 'wb') as f:
        pickle.dump(obj_to_pickle, f)
    print("Saving information to file named {}".format(file_path_and_name))


if __name__ == "__main__":
    # Use this for testing
    # This takes 0.00048 seconds
    path_to_pickle = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/hospital_trials_2021-01-15_plus_stats_clean'
    path_to_paths_file = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/paths_generated_2021-01-15_clean_data_run01'
    hosp_graph = HospGraph.unpickle_it(path_to_pickle)
    # planner = Samp.SamplingPlannerClass(hosp_graph)
    # data_accum = Samp.DataAccumulated()

    total = 0
    rooms = [i for i in range(21)]
    same = 0
    diff_cust_dij = 0
    diff_dij_alg = 0
    diff_alg_cust = 0

    diff_weighted_one = 0
    diff_one_95 = 0
    diff_95_weighted = 0

    # iterations = 1000
    sample_size = 1000
    # data_accum.paths = []

    # Looking at room pairings that would potentially have multiple paths.
    # Makes no sense to compare methods if there is only one realistic path in any situation

    options = 0

    for rm1 in range(21):
        for rm2 in range(21):
            if rm2 == rm1:
                continue

            n1 = 'r' + "{:02d}".format(rm1)
            n2 = 'r' + "{:02d}".format(rm2)

            options += 1
            alg_weighted_means = nx.dijkstra_path(hosp_graph, n1, n2, weight=Alg.weighted_means)
            alg_weighted_95_pct = nx.dijkstra_path(hosp_graph, n1, n2, weight=Alg.weighted_95_percentile)

            diff0 = [m for m in alg_weighted_95_pct if m not in alg_weighted_means]
            diff1 = [n for n in alg_weighted_means if n not in alg_weighted_95_pct]

            if len(diff0) > 1 and len(diff1) > 1:
                diff_95_weighted += 1
                print('weighted route: {}'.format(alg_weighted_means))
                print('95 pct route:   {}'.format(alg_weighted_95_pct))

    print("percent of same path:")
    print('95 percentile vs weighted: {}'.format((1 - (diff_95_weighted / options)) * 100))

    # room_pairs = [[8, 9, 10, 12, 13, 14, 15],  # 0
    #               [7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17],  # 1
    #               [11, 12, 13, 14, 15, 16, 17, 18],  # 2
    #               [11, 12, 13, 14, 15, 16, 17, 18],  # 3
    #               [11, 12, 13, 14, 15, 16, 17, 18],  # 4
    #               [11, 12, 13, 14, 15, 16, 17, 18],  # 5
    #               [17, 18, 19, 20],  # 6
    #               [17, 18, 19, 20],  # 7
    #               [0, 1, 19, 20],  # 8
    #               [0, 1, 19, 20],  # 9
    #               [0, 1, 19, 20],  # 10
    #               [0, 1, 2, 3, 4, 5, 19, 20],  # 11
    #               [0, 1, 2, 3, 4, 5, 19, 20],  # 12
    #               [0, 1, 2, 3, 4, 5, 19, 20],  # 13
    #               [0, 1, 2, 3, 4, 5, 19, 20],  # 14
    #               [0, 1, 2, 3, 4, 5, 19, 20],  # 15
    #               [0, 1, 2, 3, 4, 5, 19, 20],  # 16
    #               [0, 1, 2, 3, 4, 5, 6, 7],  # 17
    #               [0, 1, 2, 3, 4, 5, 6, 7],  # 18
    #               [6, 7, 8, 9, 10, 11, 12, 13, 14, 15],  # 19
    #               [6, 7, 8, 9, 10, 11, 12, 13, 14, 15]]  # 20
    #    for num in range(iterations):
    #     if not num % 100:
    #         print('----------- {} -------------'.format(num))
    #
    #     # Pick a random start and end room
    #     rm1 = randint(0, 20)
    #     rm2 = sample(room_pairs[rm1], 1)[0]  # This comes out as an array with one item, so we just call that item

    #     planner = Samp.SamplingPlannerClass(hosp_graph)
    #
    #     custom_time, custom = planner.get_path(sample_size, n1, n2)
    #     dijkstra = nx.dijkstra_path(hosp_graph, n1, n2, weight='euclidean_dist')
    #     dijkstra_time = nx.dijkstra_path_length(hosp_graph, n1, n2, weight='euclidean_dist')/0.22

    #     # print('path from {} to {}'.format(n1, n2))
    #     # print('custom: {}  {}'.format(custom_time, custom))
    #     # print('dijkstra: {} {}'.format(dijkstra_time, dijkstra))
    #     # print('algebraic: {} {}'.format(algebraic_time, algebraic))
    #
    #     diff = [i for i in dijkstra if i not in custom]
    #     diff2 = [j for j in custom if j not in dijkstra]
    #
    #     diff3 = [k for k in alg_weighted_means if k not in dijkstra]
    #     diff4 = [l for l in dijkstra if l not in alg_weighted_means]
    #
    #     diff5 = [m for m in custom if m not in alg_weighted_means]
    #     diff6 = [n for n in alg_weighted_means if n not in custom]
    #
    #     if len(diff) > 1 and len(diff2) > 1:
    #         diff_cust_dij += 1
    #
    #     if len(diff3) > 1 and len(diff4) > 1:
    #         diff_dij_alg += 1
    #
    #     if len(diff5) > 1 and len(diff6) > 1:
    #         diff_alg_cust += 1
    #
    #     data_accum.paths.append([custom, alg_weighted_means, dijkstra])
    #     data_accum.times.append([custom_time, alg_weighted_means_time, dijkstra_time])
    #     if not num % 200:
    #         pickle_it(data_accum, path_to_paths_file)
    #
    #     # print(time() - start_time)
    #
    # pickle_it(data_accum, path_to_paths_file)
    #
    # # print("average time: {}".format((time() - start_time) / iterations))
    # print("percent of same path:")
    # print('custom and dijkstra: {}'.format((1 - (diff_cust_dij / iterations)) * 100))
    # print('algebraic and dijkstra: {}'.format((1 - (diff_dij_alg / iterations)) * 100))
    # print('custom and algebraic: {}'.format((1 - (diff_alg_cust / iterations)) * 100))
