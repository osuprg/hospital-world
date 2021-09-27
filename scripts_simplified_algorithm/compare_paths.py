#!/usr/bin/env python3

# Python things
from time import time
import numpy as np
import pandas as pd
from statistics import mean, stdev
import pickle
from math import sqrt
from sklearn import mixture
import operator

# Custom things
import sampling_planner as Samp


class CompareMethods:
    def __init__(self, hosp_graph, iterations):
        self.iterations = iterations
        self.hosp_graph = hosp_graph
        # This will be a dictionary of dictionaries. Because the first rule of optimization club is don't optimize.
        self.df = pd.DataFrame({'path': [], 'methods': [], 'pathName': [], 'methodNames': [],
                                   'sampleData': [], 'gmmFit': [],
                                   'humDist': [], 'fourConnDist': [], 'doors': [],
                                   'gausMean': [], 'gausStd': [],
                                   'gmmData': [], 'navFail':[]})

        self.sampling_planner = Samp.SamplingPlannerClass(self.hosp_graph)

    def unique_paths(self, paths):
        # Load the return paths array with the first path in the paths array
        ret_paths = []

        for path in paths:
            if path not in ret_paths:
                ret_paths.append(path)

        return ret_paths

    def compare_paths_v2(self, paths):
        tested = 0
        path_lens = []

        # print(len(paths))
        # start_time = time()
        costs_gaus, hum_dist = self.sampling_planner.run_backward_v2(paths, self.iterations)
        # v2_time = time() - start_time

        # start_time = time()
        # costs_gaus_v3, hum_dist_v3 = self.sampling_planner.run_backward_v3(paths, self.iterations)
        # v3_time = time() - start_time
        # print('Sam2 time: {:0.05f}'.format(v2_time))
        # print('Sam3 time: {:0.05f}'.format(v3_time))

        # For each path, fit a GMM
        for i in range(len(paths)):
            path = paths[i]
            names = []
            path_len = 0
            methods = 'dummy'

            path_name = ''
            path_door_ct = 0
            nav_fail = 0
            for j in range(len(path)):
                path_name += str(path[j]) + ", "
                if j < len(path) - 1:
                    n1 = path[j]
                    n2 = path[j + 1]
                    path_len += self.hosp_graph[n1][n2]['sq_dist']
                    nav_fail += self.hosp_graph[n1][n2]['nav_fail']
                    path_door_ct += self.hosp_graph[n1][n2]['doors']

            path_lens.append(path_len)

            gmm_fit = self.try_GMM(costs_gaus[i])

            # gmm_fit_v3 = self.try_GMM(costs_gaus_v3[i])
            # Makes sure the mixture models are sorted lowest mean to highest mean
            gmm_dummy = [[gmm_fit.means_[i][0], sqrt(gmm_fit.covariances_[i]), gmm_fit.weights_[i]] for i in range(len(gmm_fit.means_))]
            gmm_data = sorted(gmm_dummy, key=operator.itemgetter(0))

            # gmm_dummy_v3 = [[gmm_fit_v3.means_[i][0], sqrt(gmm_fit_v3.covariances_[i]), gmm_fit_v3.weights_[i]] for i in range(len(gmm_fit_v3.means_))]
            # gmm_data_v3 = sorted(gmm_dummy_v3, key=operator.itemgetter(0))

            # print("%%%%%%%%%%%%%")

            self.df = self.df.append({'path': path, 'methods': methods, 'pathName': path_name, 'methodNames': names,
                                   'sampleData': costs_gaus[i], 'gmmFit': gmm_fit,
                                   'humDist': mean(hum_dist[i]), 'fourConnDist': path_len, 'doors': path_door_ct,
                                   'gausMean': mean(costs_gaus[i]), 'gausStd': stdev(costs_gaus[i]),
                                   'gmmData': gmm_data, 'navFail': nav_fail}, ignore_index=True)

    def try_GMM(self, costs_gaus):
        # costs_dummy = np.array(costs_gaus)
        costs_dummy = np.reshape(costs_gaus, (-1, 1))
        # costs_gaus_t = np.ndarray.transpose(costs_dummy)
        bayes_gmm = mixture.BayesianGaussianMixture(n_components=2)
        bayes_data = bayes_gmm.fit(costs_dummy)
        return bayes_data


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
    pass
