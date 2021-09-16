#!/usr/bin/env python3

# Python things
import networkx as nx
import numpy as np
import pandas as pd
from statistics import mean
import matplotlib.pyplot as plt

# Custom things
import STRUCT_hospital_graph_class as HospGraph
import RUN_custom_global_plan_sampling as Samp
import POST_compare_choice_methods as Choices
from scripts_simplified_algorithm import cow_goes_MOO as myMOO
import pickle

class FunctionDefs:
    def __init__(self):
        self.w_hum_gaus = 1
        self.w_no_hum_gaus = 1
        self.w_mean = 1
        self.w_errors = 1
        self.w_pct_hum = 1
        self.w_sq_dist = 1
        self.w_num_doors = 1
        self.w_hum_dist = 1
        self.w_std_no = 1
        self.w_std_hum = 1
        self.w_std_all = 1
        self.w1 = 1
        self.w2 = 1
        self.val1 = 'mean_all'
        self.val2 = '95_all'

    def fallback(self):
        # If there is no data, this is likely an infeasible edge to traverse
        return 1000

    def mean_one_gaussian(self, n1, n2, dict):
        """ Ignores human condition. Returns the mean of all data on the edge.
        If there is no data, returns the euclidean distance divided by the top speed"""

        mean_all = dict['mean_all']

        if mean_all:
            return mean_all
        else:
            return self.fallback()

    def hum_95_percentile(self, n1, n2, dict):
        pct_hum = dict['pct_hum']
        mean_hum = dict['mean_hum']
        std_hum = dict['std_hum']
        mean_no = dict['mean_no']
        std_no = dict['std_no']

        if mean_hum:
            return mean_hum + 2 * std_hum
        elif mean_no:
            return mean_no + 2 * std_no
        else:
            return self.fallback()

    def no_hum_95_percentile(self, n1, n2, dict):
        mean_no = dict['mean_no']
        std_no = dict['std_no']

        if mean_no:
            return mean_no + 2 * std_no
        else:
            return self.fallback()

    def num_doors(self, n1, n2, dict):
        cost = 0
        if 'd' in n1:
            cost += 0.5
        if 'd' in n2:
            cost += 0.5

        return cost

    def not_slow(self, n1, n2, dict):
        num_err = dict['nav_fail']
        total_cost = self.w_hum_gaus * self.hum_95_percentile(n1, n2, dict) \
                     + self.w_mean * self.mean_one_gaussian(n1, n2, dict) \
                     + self.w_errors * num_err

        return total_cost

    def fast(self, n1, n2, dict):
        return self.w_mean * self.mean_one_gaussian(n1, n2, dict)

    def fast_empty(self, n1, n2, dict):
        sq_dist = dict['sq_dist']
        total_cost = self.w_sq_dist * sq_dist \
                     + self.w_no_hum_gaus * self.no_hum_95_percentile(n1, n2, dict)
        return total_cost

    def see_humans(self, n1, n2, dict):
        pct_hum = dict['pct_hum']
        sq_dist = dict['sq_dist']
        # Inverse of percent human because we are trying to maximize human contact and Dijkstra minimizes
        total_cost = self.w_pct_hum * (1 - pct_hum) + self.w_sq_dist * sq_dist
        return total_cost

    def delicate(self, n1, n2, dict):
        pct_hum = dict['pct_hum']
        sq_dist = dict['sq_dist']
        nav_fail = dict['nav_fail']

        total_cost = self.w_hum_dist * (sq_dist * pct_hum) \
                     + self.w_num_doors * self.num_doors(n1, n2, dict) \
                     + self.w_errors * nav_fail
        return total_cost

    def variability(self, n1, n2, dict):
        mean_all = dict['mean_all']
        std_all = dict['std_all']
        num_err = dict['nav_fail']
        tot_cost = self.w_mean * mean_all + self.w_std_all * std_all + self.w_errors * num_err
        # print(n1, n2, tot_cost, self.w_mean, mean_all, self.w_std_all,  std_all, self.w_errors, num_err)
        return tot_cost

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
    hosp_graph = HospGraph.unpickle_it(hosp_graph_pickle)

    """
    Dijkstra can accept function calls as a weight. From documentation:
    If this is a function, the weight of an edge is the value returned by the function.
    The function must accept exactly three positional arguments:
    the two endpoints of an edge and the dictionary of edge attributes for that edge.
    The function must return a number.
    """
    fun_defs = FunctionDefs()

    sampling_planner = Samp.SamplingPlannerClass(hosp_graph)

    ratio_data_filepath = "/home/toothless/workspaces/research_ws/src/hospital-world/pickles/ratio_data_02-22-2021_run02"
    # data_accum = Samp.DataAccumulated()
    # all_paths_planner = All.AllPathsPlanner(hosp_graph)

    iterations = 1000
    sample_size = 1000

    compare = Choices.CompareMethods(hosp_graph, iterations)
    manipulate = myMOO.ManipulateDF()

    node_pairs = [('r00', 'r10'), ('r00', 'r15'), ('r01', 'r09'), ('r01', 'r15'), ('r04', 'r16')]
    # node_pairs = [('r00', 'r10')]

    path_vals = ['mean_all', 'std_all', 'mean_no', 'std_no', 'hum_dist', 'sq_dist', 'doors', 'nav_fail']
    moo_vals = ['gausMean', 'gausStd', 'gmmMean0', 'gmmSTD0', 'gmmUpper0', 'gmmLower0', 'gmmMean1', 'gmmSTD1','gmmUpper1', 'gmmLower1', 'humDist', 'fourConnDist', 'doors', 'navFail']

    all_ratios = []
    w1 = 1
    w2 = 1

    # dummy1 = [1, 10, 100]
    # dummy2 = [1, 2.5, 5, 7.5]
    # weight_options = []
    # for dum1 in dummy1:
    #     for dum2 in dummy2:
    #         weight_options.append(dum1 * dum2)

    weight_options = [1, 2, 3, 4, 5, 7.5]
    for by_ten in range(1, 11):
        weight_options.append(by_ten * 10)
    weight_options.extend([250, 500, 750, 1000, 2000])
    for rat in reversed(weight_options):
        all_ratios.append((rat, 1))

    for rat in weight_options:
        if rat ==1:
            continue
        all_ratios.append((1, rat))

    # print(all_ratios)
    dij_rat_count = np.zeros(len(all_ratios))
    moo_rat_count = np.zeros(len(all_ratios))


    moo_bif_ratios = []
    dij_bif_ratios = []

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

            # mult_paths = compare.main_loop(n1, n2)
            # if not mult_paths:
            #     continue

            # manipulate.df = compare.df
            # num_compare = 0
            # manipulate.setup_df_vals()
            all_dij_paths = []
            print('----------- {} to {} -----------'.format(n1, n2))

            for val1 in range(len(path_vals)):
                for val2 in range(len(path_vals)):
                    if val2 <= val1:
                        continue
                    fun_defs.val1 = path_vals[val1]
                    fun_defs.val2 = path_vals[val2]

                    dij_path = []
                    last_w1w2 = (0, 0)
                    for rat_num in range(len(all_ratios)):
                        (w1, w2) = all_ratios[rat_num]
                        last_dij_path = dij_path
                        fun_defs.w1 = w1
                        fun_defs.w2 = w2

                        dij_path = nx.dijkstra_path(compare.hosp_graph, n1, n2, weight=fun_defs.generic_function)
                        all_dij_paths.append(['nothing', dij_path])
                        if last_dij_path and last_dij_path != dij_path:
                            # print('BIFURCATION POINT')
                            dij_bif_ratios.append([last_w1w2, (fun_defs.w1, fun_defs.w2)])
                            dij_rat_count[rat_num] += 1
                            # if w1 >= 100 or w2 >= 100:
                            #     print('dijkstra', fun_defs.val1, fun_defs.val2, last_w1w2, (fun_defs.w1, fun_defs.w2), dij_path)
                        last_w1w2 = (fun_defs.w1, fun_defs.w2)

            unique_dij_paths = compare.unique_paths(all_dij_paths)
            if len(unique_dij_paths) == 1:
                print(unique_dij_paths[0][1])
            else:
                path_costs, hum_dist = sampling_planner.run_backward_v2(unique_dij_paths, 1000)
                for i in range(len(unique_dij_paths)):
                    print(mean(path_costs[i]), mean(hum_dist[i]), unique_dij_paths[i][1])
            # for moo_val1 in range(len(moo_vals)):
            #     for moo_val2 in range(len(moo_vals)):
            #         if moo_val2 <= moo_val1:
            #             continue
            #
            #         w1 = 1
            #         moo_path = []
            #         last_w1w2 = (0, 0)
            #
            #         for rat_num in range(len(all_ratios)):
            #             (w1, w2) = all_ratios[rat_num]
            #             last_moo_path = moo_path
            #
            #             moo_method = [[moo_vals[moo_val1], 'Min', w1], [moo_vals[moo_val2], 'Min', w2]]
            #             moo_path = manipulate.get_dominant_path(moo_method)
            #
            #             if last_moo_path and last_moo_path != moo_path:
            #                 # print('BIFURCATION POINT')
            #                 moo_bif_ratios.append([last_w1w2, (w1, w2)])
            #                 moo_rat_count[rat_num] += 1
            #                 if w1 >= 100 or w2 >= 100:
            #                     print('MOO', moo_vals[moo_val1], moo_vals[moo_val2], last_w1w2, (w1, w2), moo_path)
            #             last_w1w2 = (w1, w2)


    dij_norm = sum(dij_rat_count)
    dij_rat_norm = dij_rat_count / dij_norm
    moo_norm = sum(moo_rat_count)
    moo_rat_norm = moo_rat_count / moo_norm

    ratio_df = pd.DataFrame({'ratios': all_ratios, 'dijkstra_norm': dij_rat_norm, 'dijkstra_ct': dij_rat_count,
                             'moo_norm': moo_rat_norm, 'moo_ct': moo_rat_count})
    print(ratio_df)

    pickle_it(ratio_df, ratio_data_filepath)

    ax = ratio_df.plot.bar('ratios', ['dijkstra_norm', 'moo_norm'])
    ax.set_xticklabels(ratio_df.ratios)
    plt.tight_layout()
    plt.show()














    # node_pairs = [('r00', 'r10'), ('r00', 'r15'), ('r04', 'r16')]
    # node_pairs = [('r01', 'r09'), ('r01', 'r15')]
    # lw_bound = 1
    # up_bound = 50
    # step_by = 5
    #
    # up_range = int(up_bound / step_by)
    # w_errors_arr = [1, 5, 10, 25, 50, 100, 250, 500, 1000]
    #
    #
    # for (n1, n2) in node_pairs:
    #     for compare_meth in range(2):
    #         mult_paths = compare.main_loop(n1, n2)
    #         manipulate.df = compare.df
    #         manipulate.setup_df_vals()
    #         print(manipulate.df['pathName'])
    #
    #         for w in range(lw_bound, up_range):
    #             # w_mean_all = step_by * weight1
    #             # w_std_all = up_bound - w_mean_all
    #             weight1 = step_by * w
    #             weight2 = up_bound - weight1
    #             weight_fun = fun_defs.delicate
    #
    #             if compare_meth == 0 :
    #                 print('---- Dijkstra ----')
    #                 for w_err in w_errors_arr:
    #                     # fun_defs.w_mean = w_mean_all
    #                     # fun_defs.w_std_all = w_std_all
    #                     fun_defs.w_errors = w_err
    #                     fun_defs.w_hum_dist = weight1
    #                     fun_defs.w_num_doors = weight2
    #
    #                     path = dijkstra_path(hosp_graph, n1, n2, weight=weight_fun)
    #                     print('mean: {} | var: {} | error: {} | {}'.format(weight1, weight2, w_err, path))
    #
    #             elif compare_meth == 1:
    #                 print('---- MOO ----')
    #
    #                 for w_err in w_errors_arr:
    #                     methods = [['humDist', 'Min', weight1], ['doors', 'Min', weight2], ['navFail', 'Min', w_err]]
    #                     if mult_paths:
    #                         MOO_path = manipulate.get_dominant_path(methods)
    #                         print('hum: {} | doors: {} | error: {} | {}'.format(weight1, weight2, w_err, MOO_path))
    #             else:
    #                 raise ValueError('something went wrong')
    #             # print(w_hum_dist, w_num_doors, w_err)
    #             print("#########")
