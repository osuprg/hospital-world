#!/usr/bin/env python3

# Python things
from time import time
import networkx as nx
from random import random, randint, sample
import numpy as np
import pandas as pd
from statistics import mean, stdev, StatisticsError
import pickle
from math import sqrt
from sklearn import mixture
import operator

# Custom things
import STRUCT_hospital_graph_class as HospGraph
import RUN_custom_global_plan_algebraic as Alg
import RUN_custom_global_plan_sampling as Samp
import RUN_custom_global_plan_all_paths as All
import POST_plot_stacked_gaussians as PltGaus


class MethodDef:
    def __init__(self, name, function, custom=False, ignore_hum=False):
        self.name = name
        self.function = function
        self.paths = []
        self.means = []
        self.stds = []
        self.max_count = 0
        self.custom = custom
        self.ignore_hum = ignore_hum


class CompareMethods:
    def __init__(self, hosp_graph, iterations):

        # Define all the weight functions you want to use.
        self.four_conn = MethodDef('square_dist', 'four_connect_dist')
        self.alg_means_hum = MethodDef('means', Alg.mean_one_gaussian)
        self.alg_95_pct_combo = MethodDef('95_pct_mix', Alg.weighted_95_percentile)
        self.alg_std_hum = MethodDef('std_and_hum', Alg.std_and_hum)
        self.alg_95_pct_hum = MethodDef('95_pct_hum', Alg.hum_95_percentile)
        self.alg_doors = MethodDef('num_doors', Alg.num_doors)
        self.alg_four_conn_hum = MethodDef('four_conn_hum', Alg.four_conn_hum)
        self.four_conn_no = MethodDef('square_dist_no', 'four_connect_dist', ignore_hum=True)
        self.alg_95_pct_no = MethodDef('95_pct_no_hum', Alg.no_hum_95_percentile, ignore_hum=True)
        self.alg_mean_no = MethodDef('mean_no_hum', Alg.mean_no_hum, ignore_hum=True)

        # self.methods_no = [self.four_conn_no, self.alg_mean_no, self.alg_95_pct_no]
        self.methods = [self.four_conn, self.alg_means_hum, self.alg_mean_no, self.alg_95_pct_hum, self.alg_95_pct_no,
                        self.alg_95_pct_combo, self.alg_std_hum, self.alg_doors, self.alg_four_conn_hum]

        self.iterations = iterations

        self.hosp_graph = hosp_graph
        # This will be a dictionary of dictionaries. Because the first rule of optimization club is don't optimize.
        self.df = pd.DataFrame({'path': [], 'methods': [], 'pathName': [], 'methodNames': [],
                                   'sampleData': [], 'gmmFit': [],
                                   'humDist': [], 'fourConnDist': [],
                                   'gausMean': [], 'gausStd': [],
                                   'gmmData': []})

        self.sampling_planner = Samp.SamplingPlannerClass(self.hosp_graph)

    def main_loop(self, n1, n2, compare_methods=False, compare_paths=True):

        print('--------- {} to {} ---------'.format(n1, n2))

        paths = []
        # print("{}".format(self.all_methods_names[i]))

        for method in self.methods:
            if method.custom:
                path = method.function(n1, n2)
            else:
                path = nx.dijkstra_path(self.hosp_graph, n1, n2, weight=method.function)

            method.paths.append(path)
            paths.append([method, path])

            if compare_methods:
                self.compare_methods(path, method)

        if compare_paths:
            unique_paths = self.unique_paths(paths)
            if len(unique_paths) > 1:
                self.compare_paths_v2(unique_paths)
            else:
                print(unique_paths[0][1])


    def unique_paths(self, paths):
        # Load the return paths array with the first path in the paths array
        [method0, path0] = paths[0]
        ret_paths = [[[method0], path0]]

        # Loop through the paths array
        for i in range(len(paths)):
            in_list = False
            # We can skip the first path since we already added it
            if i == 0:
                continue

            # Loop through the array of paths we're going to return
            for j in range(len(ret_paths)):
                # Compare to see the path in paths is already on the return paths list
                diff0 = [m for m in paths[i][1] if m not in ret_paths[j][1]]
                diff1 = [n for n in ret_paths[j][1] if n not in paths[i][1]]

                # If there is little or no difference between the two, it's already on the list
                if len(diff0) <= 1 and len(diff1) <= 1:
                    in_list = True
                    ret_paths[j][0].append(paths[i][0])

            if not in_list:
                ret_paths.append([[paths[i][0]], paths[i][1]])

        return ret_paths

    def compare_paths_v2(self, paths):
        start_time = time()
        tested = 0
        # costs_real = self.sampling_planner.run_backward_v2(paths, iterations, real_data=True)
        costs_gaus, hum_dist = self.sampling_planner.run_backward_v2(paths, self.iterations, real_data=False)
        path_lens = []

        # For each path, fit a GMM
        for i in range(len(paths)):
            [methods, path] = paths[i]
            names = ''
            path_len = 0
            for method in methods:
                names += str(method.name) + ','

            path_name = ''
            for j in range(len(path)):
                path_name += str(path[j]) + ", "
                if j < len(path) - 1:
                    n1 = path[j]
                    n2 = path[j + 1]
                    path_len += self.hosp_graph[n1][n2]['four_connect_dist']

            path_lens.append(path_len)
            gmm_fit = self.try_GMM(costs_gaus[i])
            # print(names, path)
            # Makes sure the mixture models are sorted lowest mean to highest mean
            gmm_dummy = [[gmm_fit.means_[i][0], sqrt(gmm_fit.covariances_[i]), gmm_fit.weights_[i]] for i in range(len(gmm_fit.means_))]
            gmm_data = sorted(gmm_dummy, key=operator.itemgetter(0))

            this_key = 'key' + str(i)
            self.df = self.df.append({'path': path, 'methods': methods, 'pathName': path_name, 'methodNames': names,
                                   'sampleData': costs_gaus[i], 'gmmFit': gmm_fit,
                                   'humDist': mean(hum_dist[i]), 'fourConnDist': path_len,
                                   'gausMean': mean(costs_gaus[i]), 'gausStd': stdev(costs_gaus[i]),
                                   'gmmData': gmm_data}, ignore_index=True)

            # print(names)
            # print(path_name)
            # print('total path length: {} | path with humans: {}'.format(path_lens[i], mean(hum_dist[i])))
            # print('interval: [{:4.2f}, {:4.2f}]'.format(min(costs_gaus[i]), max(costs_gaus[i])))
            # print('Gauss: {:.2f} {:.2f}'.format(mean(costs_gaus[i]), stdev(costs_gaus[i])))
            # print('GMM1 : {:.2f}, {:.2f}, {:.2f}'.format(gmm_data[0][0], gmm_data[0][1], gmm_data[0][2]))
            # print('GMM2 : {:.2f}, {:.2f}, {:.2f}'.format(gmm_data[1][0], gmm_data[1][1], gmm_data[1][2]))
            # print('----')

    def compare_paths(self, paths):
        iterations = 3000
        tot_time = 0
        tested = 0
        for [methods, path] in paths:
            start_time = time()
            path_avg_samp, path_std_samp, low_samp, high_samp = self.sampling_planner.run_backward(path, iterations, methods[0].ignore_hum)
            tot_time += time() - start_time
            tested += 1

            start_time = time()
            path_avg_real, path_std_real, low_real, high_real = self.sampling_planner.run_backward(path, iterations, methods[0].ignore_hum, real_data=True)
            tot_time += time() - start_time
            tested += 1

            path_avg_gaus, path_std_gaus, low_gaus, high_gaus = self.sampling_planner.gaussian_combo(path, methods[0].ignore_hum)
            names_str = ''
            for i in range(len(methods)):
                names_str += methods[i].name + ', '

            print('{:45} {}'.format(names_str, path))

            print('samp, gaus:  [{:0.2f}, {:0.2f}], {:0.2f}, {:0.2f}'.format(low_samp, high_samp, path_avg_samp, path_std_samp))
            print('samp, real:  [{:0.2f}, {:0.2f}], {:0.2f}, {:0.2f}'.format(low_real, high_real, path_avg_real, path_std_real))
            print('gaussian:    [{:0.2f}, {:0.2f}], {:0.2f}, {:0.2f}'.format(low_gaus, high_gaus, path_avg_gaus, path_std_gaus))
        print('average time:', tot_time / tested)

    def compare_methods(self, path, method):
        path_avg, path_std, low, high = self.sampling_planner.run_backward(path, iterations,
                                                                      method.ignore_hum)

        method.means.append(path_avg)
        method.stds.append(path_std)

        print('{0:35}, [{1:0.2f}, {2:0.2f}], {3:0.2f}, {4:0.2f}, {5}'.format(method.name, low, high,
                                                                             path_avg, path_std, path))

    def try_GMM(self, costs_gaus):
        # costs_dummy = np.array(costs_gaus)
        costs_dummy = np.reshape(costs_gaus, (-1, 1))
        # costs_gaus_t = np.ndarray.transpose(costs_dummy)
        bayes_gmm = mixture.BayesianGaussianMixture(n_components=2)
        bayes_data = bayes_gmm.fit(costs_dummy)
        return bayes_data

    def compare_path_distributions(self):

        for rm1 in range(21):
            for rm2 in range(21):
                if rm2 <= rm1:
                    continue

                n1 = 'r' + "{:02d}".format(rm1)
                n2 = 'r' + "{:02d}".format(rm2)  # self.room_pairs[rm1][rm2]
                print('--------- {} to {} ---------'.format(n1, n2))

                paths = []
                # print("{}".format(self.all_methods_names[i]))

                for method in self.methods:
                    if method.custom:
                        path = method.function(n1, n2)
                    else:
                        path = nx.dijkstra_path(self.hosp_graph, n1, n2, weight=method.function)

                    method.paths.append(path)
                    paths.append([method, path])

                unique_paths = self.unique_paths(paths)
                # print(unique_paths)

                for path in unique_paths:
                    costs_real = self.sampling_planner.run_backward_v2(path, self.iterations, real_data=True)
                    costs_gaus = self.sampling_planner.run_backward_v2(path, self.iterations, real_data=False)
                    histo_costs = [[costs_real, 'real'], [costs_gaus, 'gaus']]
                    sum_data_no = self.sampling_planner.gaussian_combo(path, hum_cond="no")
                    sum_data_hum = self.sampling_planner.gaussian_combo(path, hum_cond='hum')
                    sum_data_all = self.sampling_planner.gaussian_combo(path, hum_cond='all')
                    # print(sum_data)
                    gmm = self.try_GMM(costs_gaus)
                    gmm_a = [gmm.means_[0], sqrt(gmm.covariances_[0]), None, None]
                    gmm_b = [gmm.means_[1], sqrt(gmm.covariances_[1]), None, None]

                    stats = [[sum_data_no, 'sum_no'], [sum_data_hum, 'sum_hum'], [sum_data_all, 'sum_all'],
                             [gmm_a, 'GMM 1'], [gmm_b, 'GMM 2']]

                    # PltGaus.plot_gaussians(n1, n2, histo_costs, stats, path[0][0].name)


class AddPathInfo:
    def __init__(self, hosp_graph, experiments):
        self.hosp_graph = hosp_graph
        self.experiments = experiments
        self.check_num_paths()
        self.find_diff_paths()

    def check_num_paths(self):
        curr_len = None
        # Make sure all the experiments generated the same number of paths
        for method in self.experiments:
            if not curr_len:
                curr_len = len(method.paths)
            elif len(method.paths) != curr_len:
                raise ValueError

            for (n1, n2) in self.hosp_graph.edges():
                self.hosp_graph[n1][n2]['path_count_' + method.name] = 0

    def find_diff_paths(self):
        # Get the number of paths generated
        num_paths = len(self.experiments[0].paths)
        num_methods = len(self.experiments)

        # Loop through all paths
        for i in range(num_paths):
            # Loop through all methods
            for j in range(num_methods):
                # Compared against all other methods
                for k in range(num_methods):
                    if k <= j:
                        continue
                    mth1 = self.experiments[j]
                    mth2 = self.experiments[k]

                    # Compare to see if they are different
                    diff0 = [m for m in mth1.paths[i] if m not in mth2.paths[i]]
                    diff1 = [n for n in mth2.paths[i] if n not in mth1.paths[i]]

                    if len(diff1) > 1 and len(diff0) > 1:
                        self.add_info_to_hosp_graph(mth1, mth1.paths[i])
                        self.add_info_to_hosp_graph(mth2, mth2.paths[i])

    def add_info_to_hosp_graph(self, method, path):
        # Add to the count for the method's edge weight
        for i in range(len(path) - 1):
            n1 = path[i]
            n2 = path[i + 1]
            self.hosp_graph[n1][n2]['path_count_' + method.name] += 1
            if self.hosp_graph[n1][n2]['path_count_' + method.name] > method.max_count:
                method.max_count = self.hosp_graph[n1][n2]['path_count_' + method.name]


def pickle_it(obj_to_pickle, file_path_and_name):
    with open(file_path_and_name, 'wb') as f:
        pickle.dump(obj_to_pickle, f)
    print("Saving information to file named {}".format(file_path_and_name))


def unpickle_it(filename):
    infile = open(filename, 'rb')
    unpickled = pickle.load(infile)
    infile.close()
    return unpickled


if __name__ == "__main__":
    # Use this for testing
    # This takes 0.00048 seconds
    path_to_pickle = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/hospital_trials_2021-02-11_hum_50_70_plus_stats_clean'
    path_to_paths_file = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/paths_generated_2021-02-11_clean_run_01'
    hosp_graph = HospGraph.unpickle_it(path_to_pickle)
    # self.sampling_planner = Samp.SamplingPlannerClass(hosp_graph)

    # data_accum = Samp.DataAccumulated()
    # all_paths_planner = All.AllPathsPlanner(hosp_graph)

    iterations = 1000
    sample_size = 1000

    compare = CompareMethods(hosp_graph, iterations)


    # room_pairs = [[9, 14, 15]]
    room_pairs = [[7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17],  # 0
                  [7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17],  # 1
                  [11, 12, 13, 14, 15, 16, 17, 18],  # 2
                  [11, 12, 13, 14, 15, 16, 17, 18],  # 3
                  [11, 12, 13, 14, 15, 16, 17, 18],  # 4
                  [11, 12, 13, 14, 15, 16, 17, 18],  # 5
                  [11, 12, 13, 14, 15, 16, 17, 18, 19, 20],  # 6
                  [11, 12, 13, 14, 15, 16, 17, 18, 19, 20],  # 7
                  [17, 18, 19, 20],  # 8
                  [17, 18, 19, 20],  # 9
                  [17, 18, 19, 20],  # 10
                  [19, 20],  # 11
                  [19, 20],  # 12
                  [19, 20],  # 13
                  [19, 20],  # 14
                  [19, 20]]  # 15

    # for rm1 in range(len(self.room_pairs)):
    #     for rm2 in range(len(self.room_pairs[rm1])):
    # for rm1 in range(21):
    #     for rm2 in range(21):
    #
    #         if rm2 <= rm1:
    #             continue

            # n1 = 'r' + "{:02d}".format(rm1)
            # # n2 = 'r' + "{:02d}".format(self.room_pairs[rm1][rm2])
            # n2 = 'r' + "{:02d}".format(rm2)

    n1 = 'r00'
    n2 = 'r10'
    compare.main_loop(n1, n2)
    print(compare.df['gausMean'])

    # compare.compare_path_distributions()
    # compare.try_GMM()
    # pickle_it([methods_hum, methods_no], path_to_paths_file)

    # for (n1, n2) in hosp_graph.edges():
    #     print(n1, n2, hosp_graph[n1][n2]['mean_all'])
