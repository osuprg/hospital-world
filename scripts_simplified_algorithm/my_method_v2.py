#!/usr/bin/env python3

# Python things
import networkx as nx
import numpy as np
import pandas as pd
from statistics import mean
import matplotlib.pyplot as plt
from time import time

# Custom things
import fake_hospital_graph as HospGraph
import sampling_planner as Samp
import compare_paths as Compare
import cow_goes_MOO as myMOO
from dijkstra_options import DijkstraMethod
import fake_ngsa_method as NGSA
import pickle
import warnings
warnings.filterwarnings("ignore")

def unpickle_it(filename):
    infile = open(filename, 'rb')
    unpickled = pickle.load(infile)
    infile.close()
    return unpickled


def pickle_it(obj_to_pickle, file_path_and_name):
    with open(file_path_and_name, 'wb') as f:
        pickle.dump(obj_to_pickle, f)
    print("Saving information to file named {}".format(file_path_and_name))


if __name__ == '__main__':

    # hosp_graph_pickle = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/hospital_trials_2021-02-11_hum_50_70_plus_stats_clean'

    iterations = 1000
    sample_size = 1000

    moo_vals = ['gausMean', 'gausStd', 'gmmMean0', 'gmmSTD0', 'gmmUpper0', 'gmmLower0', 'gmmMean1', 'gmmSTD1',
                'gmmUpper1', 'gmmLower1', 'humDist', 'fourConnDist', 'doors', 'navFail']

    means_arr = [['gmmUpper1', 'Min'], ['gausMean', 'Min']]  #, ['navFail', 'Min']]
    min_hum_arr = [['gmmUpper1', 'Min'], ['doors', 'Min']]  #, ['navFail', 'Min']]
    max_hum_arr = [['humDist', 'Max'], ['fourConnDist', 'Min']]
    evening_arr = [['gmmMean0', 'Min'], ['fourConnDist', 'Min']]
    go_fast = [['gmmMean0', 'Min'], ['fourConnDist', 'Min']]

    goals = ['Delicate']  #'Not slow']  #, 'Delicate', 'After hours', 'See people'] #, 'GOFAST']
    methods = [min_hum_arr]  #means_arr]  #, min_hum_arr,  evening_arr, max_hum_arr]  # go_fast]

    # methods = []
    # goals = []
    #
    # for i in range(len(moo_vals) - 1):
    #     for j in range(len(moo_vals)):
    #         if j <= i:
    #             continue
    #         for max_min_a in ['Max', 'Min']:
    #             for max_min_b in ['Max', 'Min']:
    #                 methods.append([[moo_vals[i], max_min_a], [moo_vals[j], max_min_b]])
    #                 goals.append('{}_{}'.format(moo_vals[i], moo_vals[j]))
    # print(methods)

    # node_options = [[30, 5]]
    node_options = [[16, 4], [20, 4], [25, 5], [30, 5], [35, 5], [42, 7], [49, 7],
                    [56, 7], [63, 7], [64, 8], [72, 8], [81, 9], [90, 9], [99, 9],
                    [100, 10], [110, 10], [121, 11], [130, 10], [140, 10], [150, 10]]
    saw_in_ngsa = 0
    total_methods = 0
    for [tot_nodes, loop_by] in node_options:
        fake_hosp_graph_pickle = '/home/toothless/workspaces/research_ws/src/hospital-world/scripts_simplified_algorithm/' \
                                 'fake_hospital_graphs/fake_hospital_graph_{}_nodes'.format(tot_nodes)
        hosp_graph = HospGraph.unpickle_it(fake_hosp_graph_pickle)
        sampling_planner = Samp.SamplingPlannerClass(hosp_graph)
        compare = Compare.CompareMethods(hosp_graph, iterations)
        dijk = DijkstraMethod(compare.hosp_graph)
        ngsa = NGSA.NgsaMethod()
        manipulate = myMOO.ManipulateDF()

        nodes_to_compare = [[0, tot_nodes - 1], [loop_by - 1, tot_nodes - loop_by]]

        for [n1, n2] in nodes_to_compare:
            node1_node2 = str(n1) + '_' + str(n2)
            print(node1_node2)

            # Re-initialize dataframe
            compare.df = pd.DataFrame({'path': [], 'methods': [], 'pathName': [], 'methodNames': [],
                                       'sampleData': [], 'gmmFit': [],
                                       'humDist': [], 'fourConnDist': [], 'doors': [],
                                       'gausMean': [], 'gausStd': [],
                                       'gmmData': [], 'navFail': []})
            dij_paths = dijk.janky_dijkstra(n1, n2)
            unique_dij_paths = []

            for path in dij_paths:
                if path not in unique_dij_paths:
                    unique_dij_paths.append(path)

            if len(unique_dij_paths) == 1:
                print(unique_dij_paths[0][1])
            else:
                compare.compare_paths_v2(unique_dij_paths)
                ngsa.df = compare.df
                manipulate.df = compare.df
                manipulate.setup_df_vals()

                for i in range(len(methods)):

                    moo_method = methods[i]
                    # moo_path = manipulate.get_dominant_path(moo_method)
                    # ngsa_paths = ngsa.determine_dominance(moo_method)
                    ngsa_paths = ngsa.determine_dominance_v2(moo_method)
                    ngsa.ngsa_graph(moo_method, node1_node2)

