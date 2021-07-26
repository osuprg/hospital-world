#!/usr/bin/env python3

import networkx as nx
from sklearn import mixture

from network_structure import SimGraph
from dijkstra_varying_weights import run_dijkstra
from random import random, gauss
import numpy as np
from math import sqrt


def path_gmm_distros(tst_graph, path, range_num):
    """
    Takes in a NetworkX graph and a path
    Provides a GMM distributions for the given attribute set for that path
    """

    # attr_list = list(list(tst_graph.G.edges(data=True))[0][-1].keys())
    # num_ranges = len(tst_graph.ranges_list)

    float_num_00 = 'fl00_' + '%02d' % range_num
    float_num_01 = 'fl01_' + '%02d' % range_num
    std_num_00 = 'std00_' + '%02d' % range_num
    std_num_01 = 'std01_' + '%02d' % range_num

    path_times = []

    for _ in range(1000):
        path_time = 0

        for i in range(len(path) - 1):
            n1 = path[i]
            n2 = path[i + 1]

            gmm_mean_00 = tst_graph.G[n1][n2][float_num_00]
            gmm_mean_01 = tst_graph.G[n1][n2][float_num_01]
            gmm_std_00 = tst_graph.G[n1][n2][std_num_00]
            gmm_std_01 = tst_graph.G[n1][n2][std_num_01]

            if random() < 0.5:
                path_time += gauss(gmm_mean_00, gmm_std_00)
            else:
                path_time += gauss(gmm_mean_01, gmm_std_01)

        # print(path_time)
        path_times.append(path_time)

    gmm = try_GMM(path_times)
    gmm_a = [gmm.means_[0][0], sqrt(gmm.covariances_[0]), None, None]
    gmm_b = [gmm.means_[1][0], sqrt(gmm.covariances_[1]), None, None]
    print(gmm_a)
    print(gmm_b)
    # print(attr[-2:])


def try_GMM(path_times_list):
    # costs_dummy = np.array(costs_gaus)
    costs_dummy = np.reshape(path_times_list, (-1, 1))
    # costs_gaus_t = np.ndarray.transpose(costs_dummy)
    bayes_gmm = mixture.BayesianGaussianMixture(n_components=2)
    bayes_data = bayes_gmm.fit(costs_dummy)
    return bayes_data


if __name__ == "__main__":
    e_list = [(0, 1), (0, 3), (1, 2), (1, 4),
              (3, 2), (3, 4), (2, 5), (4, 5)]
    test_graph = SimGraph(e_list)
    good_paths = run_dijkstra(test_graph)
    range_n = 2
    path_gmm_distros(test_graph, good_paths[0], range_n)

