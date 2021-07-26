#!/usr/bin/env python3

from sklearn import mixture

from random import random, gauss
import numpy as np
from math import sqrt
import plot_gmms
import matplotlib.pyplot as plt


def gmm_sample(models, fig_num):
    iter_num = 10000
    path_costs = []

    # Generate iter_num number of path costs
    for i in range(iter_num):
        # Initialize this iteration's path cost
        path_cost = 0

        # Loop through all the individual mixture models
        for gmm_model in models:
            # Choose a random number in [0, 1]
            rand_num = random()

            # For each distribution in the model
            for gmm_distro in gmm_model:
                # If the random number is less than the distribution's weight, where the weight is the sum of all
                # distribution's weights so far
                if rand_num < gmm_distro[3]:
                    # Then sample from the distribution and add it to the path cost, then skip to the next model
                    path_cost += gauss(gmm_distro[0], gmm_distro[1])
                    break

        # Save all path costs after looping through each individual model.
        path_costs.append(path_cost)

    return path_costs


def plot_histo(data, fig_num):
    plt.clf()
    plt.hist(data, bins=50, color="Purple", density=True)
    plt.savefig('{}_costs'.format(fig_num))


def try_GMM(path_times_list):
    # costs_dummy = np.array(costs_gaus)
    costs_dummy = np.reshape(path_times_list, (-1, 1))
    # costs_gaus_t = np.ndarray.transpose(costs_dummy)
    bayes_gmm = mixture.BayesianGaussianMixture(n_components=2)
    bayes_data = bayes_gmm.fit(costs_dummy)
    return bayes_data


def gmm_add_weighted(gmmA, gmmB):

    """
    Add the means and square roots together cross the large / small distributions
    """
    models = []
    wt_total = 0

    for gmm_a in gmmA:
        for gmm_b in gmmB:
            mean = gmm_a[0] + gmm_b[0]
            std = sqrt(gmm_a[1]**2 + gmm_b[1]**2)
            wt = gmm_a[2] * gmm_b[2]
            wt_total += wt
            models.append([mean, std, wt, round(wt_total, 8)])

    return models


def run_all_methods(gmm_set, num):

    print('Weighted')
    gmm_A = gmm_set[0]

    for n in range(len(gmm_set) - 1):
        gmm_B = gmm_set[n+1]
        gmm_A = gmm_add_weighted(gmm_A, gmm_B)

    weighted = gmm_A
    # for j in weighted:
    #     print(j)

    print('Sample')
    path_costs = gmm_sample(gmm_set, num)
    gmm_fit = try_GMM(path_costs)
    # print(gmm_fit.means_)
    # print([sqrt(var) for var in gmm_fit.covariances_])
    sample = []
    curr_weight = 0

    for i in range(len(gmm_fit.means_)):
        curr_weight += gmm_fit.weights_[i]
        sample.append([gmm_fit.means_[i][0], sqrt(gmm_fit.covariances_[i]), gmm_fit.weights_[i], curr_weight])

    # gmm_b = [gmm_fit.means_[1][0], sqrt(gmm_fit.covariances_[1]), gmm_fit.weights_[1], 1]
    # sample = [gmm_a, gmm_b]

    for k in sample:
        print(k)

    plot_histo(path_costs, '0_to_7')
    plot_gmms.plot_it(weighted, '{}_weighted'.format(num))
    plot_gmms.plot_it(sample, '{}_sample'.format(num))


if __name__ == "__main__":
    # GMMs represented by mean, stdev, weight
    # gmm_00 = [[5.0, 1.0, 0.3, 0.3], [10.0, 1.5, 0.7, 1.0]]
    # gmm_01 = [[2.0, 0.4, 0.5, 0.5], [22.0, 1.3, 0.5, 1.0]]
    #
    # gmm_02 = [[15.0, 2.0, 0.1, 0.1], [93.3, 5.0, 0.9, 1.0]]
    # gmm_03 = [[11.2, 0.6, 0.8, 0.8], [117.5, 1.7, 0.2, 1.0]]
    #
    # gmm_04 = [[9.2, 1.2, 0.6, 0.6], [67.2, 8.3, 0.4, 1.0]]
    # gmm_05 = [[78.1, 4.5, 0.4, 0.4], [17.2, 2.4, 0.6, 1.0]]
    #
    # gmm_06 = [[81.2, 9.6, 0.8, 0.8], [7.5, 0.7, 0.2, 1.0]]
    # gmm_07 = [[72.0, 3.4, 0.9, 0.9], [6.3, 11.0, 0.1, 1.0]]

    gmm_00 = [[5.0, 0.6, 0.5, 0.5], [30.0, 1.5, 0.5, 1.0]]
    gmm_01 = [[7.0, 0.4, 0.5, 0.5], [12.0, 1.3, 0.5, 1.0]]

    gmm_02 = [[15.0, 2.0, 0.5, 0.5], [3.3, 5.0, 0.5, 1.0]]
    gmm_03 = [[11.2, 0.6, 0.5, 0.5], [17.5, 1.7, 0.5, 1.0]]

    gmm_04 = [[9.2, 1.2, 0.5, 0.5], [27.2, 8.3, 0.5, 1.0]]
    gmm_05 = [[38.1, 4.5, 0.5, 0.5], [17.2, 2.4, 0.5, 1.0]]

    gmm_06 = [[21.2, 9.6, 0.5, 0.5], [7.5, 0.7, 0.5, 1.0]]
    gmm_07 = [[23.0, 3.4, 0.5, 0.5], [6.3, 11.0, 0.5, 1.0]]

    gmms = [gmm_00, gmm_01, gmm_02, gmm_03, gmm_04, gmm_05, gmm_06, gmm_07]

    run_all_methods(gmms, '0_to_7')
