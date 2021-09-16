#!/usr/bin/env python3

# Python things
import networkx as nx
import pandas as pd

# Custom things
import STRUCT_hospital_graph_class as HospGraph
import RUN_custom_global_plan_sampling as Samp
import POST_compare_choice_methods as Choices
from scripts_simplified_algorithm import cow_goes_MOO as myMOO
import pickle

class DijkstraMethod:
    """
    Dijkstra can accept function calls as a weight. From documentation:
    If this is a function, the weight of an edge is the value returned by the function.
    The function must accept exactly three positional arguments:
    the two endpoints of an edge and the dictionary of edge attributes for that edge.
    The function must return a number.

    In order to adjust this to also take in weights and different cost values, we wrap it
    in a class.
    """
    def __init__(self):
        self.w1 = 1
        self.w2 = 1
        self.val1 = 'mean_all'
        self.val2 = '95_all'

    def generic_function(self, n1, n2, dict):
        return self.w1 * dict[self.val1] + self.w2 * dict[self.val2]


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

    hosp_graph_pickle = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/hospital_trials_2021-02-11_hum_50_70_plus_stats_clean'
    ratio_data_filepath = "/home/toothless/workspaces/research_ws/src/hospital-world/pickles/ratio_data_02-22-2021_run02"
    iterations = 1000
    sample_size = 1000

    hosp_graph = HospGraph.unpickle_it(hosp_graph_pickle)
    fun_defs = DijkstraMethod()
    sampling_planner = Samp.SamplingPlannerClass(hosp_graph)
    compare = Choices.CompareMethods(hosp_graph, iterations)
    manipulate = myMOO.ManipulateDF()

    moo_vals = ['gausMean', 'gausStd', 'gmmMean0', 'gmmSTD0', 'gmmUpper0', 'gmmLower0', 'gmmMean1', 'gmmSTD1','gmmUpper1', 'gmmLower1', 'humDist', 'fourConnDist', 'doors', 'navFail']
    path_vals = ['mean_all', 'std_all', 'mean_no', 'std_no', 'pct_hum', 'sq_dist', 'doors', 'nav_fail']
    weight_options = [1, 5, 10, 100, 1000]
    means_arr = [['gmmUpper1', 'Min', 10], ['gausMean', 'Min', 5], ['navFail', 'Min', 1]]
    min_hum_arr = [['humDist', 'Min', 10], ['doors', 'Min', 2]]  #, ['navFail', 'Min', 1]]
    max_hum_arr = [['humDist', 'Max', 10], ['fourConnDist', 'Min', 10]]
    evening_arr = [['gmmMean0', 'Min', 10], ['fourConnDist', 'Min', 5]]
    go_fast = [['gmmMean0', 'Min', 10], ['fourConnDist', 'Min', 5]]

    goals = ['Not slow', 'Delicate', 'After hours'] #, 'See people', 'GOFAST']
    methods = [means_arr, min_hum_arr,  evening_arr] #, max_hum_arr, go_fast]

    all_ratios = []
    w1 = 1
    w2 = 1

    # Get all ratio pairs
    for rat in reversed(weight_options):
        all_ratios.append((rat, 1))
    for rat in weight_options:
        if rat == 1:
            continue
        all_ratios.append((1, rat))

    # For testing one-off paths
    # fun_defs.w1 = 1
    # fun_defs.w2 = 0
    # fun_defs.val1 = 'sq_dist'
    # fun_defs.val2 = 'mean_all'
    #
    # n1 = 'r00'
    # n2 = 'r10'
    # dij_path = nx.dijkstra_path(compare.hosp_graph, n1, n2, weight=fun_defs.generic_function)
    # print(dij_path)

    # for rm1 in [0]:  # , 1, 2]:
    #     for rm2 in [10]:  # , 11, 12, 13, 14, 15, 16]:

            # print(all_ratios)
    # Loop through all room pairs
    for rm1 in range(21):
        for rm2 in range(21):
            if rm2 <= rm1:
                continue

            n1 = 'r' + "{:02d}".format(rm1)
            n2 = 'r' + "{:02d}".format(rm2)

            # Re-initialize dataframe
            compare.df = pd.DataFrame({'path': [], 'methods': [], 'pathName': [], 'methodNames': [],
                                       'sampleData': [], 'gmmFit': [],
                                       'humDist': [], 'fourConnDist': [], 'doors': [],
                                       'gausMean': [], 'gausStd': [],
                                       'gmmData': [], 'navFail': []})

            all_dij_paths = []
            print('----------- {} to {} -----------'.format(n1, n2))

            # Loop through all pairs of path values
            for val1 in range(len(path_vals)):
                for val2 in range(len(path_vals)):
                    if val2 <= val1:
                        continue

                    # Set the values in the class
                    fun_defs.val1 = path_vals[val1]
                    fun_defs.val2 = path_vals[val2]

                    # Finally loop through all ratios
                    for rat_num in range(len(all_ratios)):
                        # Get the current set of ratios and set the class ratios
                        (w1, w2) = all_ratios[rat_num]
                        fun_defs.w1 = w1
                        fun_defs.w2 = w2

                        # Function name is a combo of the value names and weights, for posterity
                        fun_name = str(path_vals[val1]) + str(w1) + str(path_vals[val2]) + str(w2)
                        # Get the path based on the current ratio
                        dij_path = nx.dijkstra_path(compare.hosp_graph, n1, n2, weight=fun_defs.generic_function)

                        all_dij_paths.append([fun_name, dij_path])

            unique_dij_paths = compare.unique_paths(all_dij_paths)

            if len(unique_dij_paths) == 1:
                print(unique_dij_paths[0][1])
            else:
                compare.compare_paths_v2(unique_dij_paths)
                manipulate.df = compare.df

                num_compare = 0
                manipulate.setup_df_vals()

                for i in range(len(methods)):
                    moo_method = methods[i]
                    moo_path = manipulate.get_dominant_path(moo_method)
                    print("[{:25} [{}]]".format(goals[i], moo_path))


