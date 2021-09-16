#!/usr/bin/env python3

# Python things
import networkx as nx
import numpy as np
import pandas as pd
from statistics import mean
import matplotlib.pyplot as plt

# Custom things
import fake_hospital_graph as HospGraph
import sampling_planner as Samp
import compare_paths as Compare
import cow_goes_MOO as myMOO
from dijkstra_options import DijkstraMethod
import fake_ngsa_method as NGSA
import pickle

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
    # ratio_data_filepath = "/home/toothless/workspaces/research_ws/src/hospital-world/pickles/ratio_data_02-22-2021_run02"

    fake_hosp_graph_pickle = '/home/toothless/workspaces/research_ws/src/hospital-world/scripts_simplified_algorithm/fake_hospital_graphs/fake_hospital_graph_50_nodes'
    iterations = 1000
    sample_size = 1000

    hosp_graph = HospGraph.unpickle_it(fake_hosp_graph_pickle)
    sampling_planner = Samp.SamplingPlannerClass(hosp_graph)
    compare = Compare.CompareMethods(hosp_graph, iterations)
    dijk = DijkstraMethod(compare.hosp_graph)
    ngsa = NGSA.NgsaMethod()
    manipulate = myMOO.ManipulateDF()

    moo_vals = ['gausMean', 'gausStd', 'gmmMean0', 'gmmSTD0', 'gmmUpper0', 'gmmLower0', 'gmmMean1', 'gmmSTD1',
                'gmmUpper1', 'gmmLower1', 'humDist', 'fourConnDist', 'doors', 'navFail']

    means_arr = [['gmmUpper1', 'Min'], ['gausMean', 'Min'], ['navFail', 'Min']]
    min_hum_arr = [['gmmUpper1', 'Min'], ['doors', 'Min']]  #, ['navFail', 'Min']]
    max_hum_arr = [['humDist', 'Max'], ['fourConnDist', 'Min']]
    evening_arr = [['gmmMean0', 'Min'], ['fourConnDist', 'Min']]
    go_fast = [['gmmMean0', 'Min'], ['fourConnDist', 'Min']]

    goals = ['Not slow', 'Delicate', 'After hours'] #, 'See people', 'GOFAST']
    methods = [means_arr, min_hum_arr,  evening_arr] #, max_hum_arr, go_fast]

    for n1 in [0]:
        for n2 in [49]:
            # Re-initialize dataframe
            compare.df = pd.DataFrame({'path': [], 'methods': [], 'pathName': [], 'methodNames': [],
                                       'sampleData': [], 'gmmFit': [],
                                       'humDist': [], 'fourConnDist': [], 'doors': [],
                                       'gausMean': [], 'gausStd': [],
                                       'gmmData': [], 'navFail': []})

            dij_paths = dijk.janky_dijkstra(n1, n2)
            unique_dij_paths = compare.unique_paths(dij_paths)

            if len(unique_dij_paths) == 1:
                print(unique_dij_paths[0][1])
            else:
                compare.compare_paths_v2(unique_dij_paths)
                ngsa.df = compare.df
                manipulate.df = compare.df
                manipulate.setup_df_vals()

                for i in range(len(methods)):
                    print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
                    moo_method = methods[i]
                    moo_path = manipulate.get_dominant_path(moo_method)
                    print("[{:25} [{}]]".format(goals[i], moo_path))
                    ngsa_path = ngsa.determine_dominance(moo_method)
                    # print("[{:25} [{}]]".format(' ', ngsa_path))


    # Loop through all room pairs
    # for rm1 in range(21):
    #     for rm2 in range(21):
    #         if rm2 <= rm1:
    #             continue
    # for rm1 in [0]:  # , 1, 2]:
    #     for rm2 in [10]:  # , 11, 12, 13, 14, 15, 16]:
    #         n1 = 'r' + "{:02d}".format(rm1)
    #         n2 = 'r' + "{:02d}".format(rm2)
