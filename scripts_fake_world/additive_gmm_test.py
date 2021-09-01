#!/usr/bin/env python3

from sklearn import mixture

from random import random, gauss, uniform
import numpy as np
from math import sqrt
import plot_gmms
import matplotlib.pyplot as plt
from timeit import timeit
from time import time
from datetime import date
from atexit import register

iter_num = 10000


def gmm_sample(models):
    path_costs = []

    # Generate iter_num number of path costs
    for i in range(iter_num):
        # Initialize this iteration's path cost
        path_cost = 0

        # Loop through all the individual mixture models, each associated with an edge
        for gmm_model in models:
            # Choose a random number in [0, 1]
            rand_num = random()

            # For each distribution in the model
            for gmm_distro in gmm_model:
                # If the random number is less than the distribution's weight, where the weight is the sum of all
                # distribution's weights so far
                if rand_num < gmm_distro[3]:
                    # Then sample from the distribution and add it to the path cost, then skip to the next edge
                    path_cost += gauss(gmm_distro[0], gmm_distro[1])
                    break

        # Save all path costs after looping through each individual model.
        path_costs.append(path_cost)

    return path_costs


def fit_gmm(path_times_list, num_components=2):
    # costs_dummy = np.array(costs_gaus)
    costs_dummy = np.reshape(path_times_list, (-1, 1))
    # costs_gaus_t = np.ndarray.transpose(costs_dummy)
    bayes_gmm = mixture.BayesianGaussianMixture(n_components=num_components)
    bayes_data = bayes_gmm.fit(costs_dummy)

    return bayes_data


def gmm_reformat(gmm_fit):
    curr_weight = 0
    reformat = []

    for i in range(len(gmm_fit.means_)):
        curr_weight += gmm_fit.weights_[i]
        reformat.append([gmm_fit.means_[i][0], sqrt(gmm_fit.covariances_[i]), gmm_fit.weights_[i], curr_weight])

    return reformat


def gmm_add_weighted(gmmA, gmmB):
    """
    Add the means and standard deviations together cross the large / small distributions
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


def resample_gmms(model_set):
    """
    takes in a set of GMM models and resamples then refits them to only 3 models
    """
    samples = np.zeros(iter_num)

    for i in range(iter_num):
        rand_num = random()
        # For each distribution in the model
        for gmm_distro in model_set:
            # If the random number is less than the distribution's weight, where the weight is the sum of all
            # distribution's weights so far
            if rand_num < gmm_distro[3]:
                # Then sample from the distribution and save it as the path cost, then skip to the next iteration
                samples[i] = gauss(gmm_distro[0], gmm_distro[1])
                break

    # plt.hist(samples, bins=50, density=True)
    # plt.show()

    return samples


def their_method(models):
    gmm_A = models[0]

    for n in range(len(models) - 1):
        gmm_B = models[n + 1]
        gmm_A = gmm_add_weighted(gmm_A, gmm_B)
        if len(gmm_A) > 10:
            resample = resample_gmms(gmm_A)
            gmm_A = gmm_reformat(fit_gmm(resample))

    if len(gmm_A) > 2:
        resample = resample_gmms(gmm_A)
        weighted = gmm_reformat(fit_gmm(resample))
    else:
        weighted = gmm_A

    return weighted


def mcmc(model_set):
    path_costs = gmm_sample(model_set)
    sample = gmm_reformat(fit_gmm(path_costs))
    return sample


def run_all_methods(gmm_set, num):

    theirs = their_method(gmm_set)

    mine = mcmc(gmm_set)

    plot_gmms.plot_it(theirs, '{}_weighted'.format(num))
    plot_gmms.plot_it(mine, '{}_sample'.format(num))


def generate_edge_data(num_edges, mu_range, std_range):
    # each edge will have two models, each model contains four components [mu, std, weight, total weight]
    fake_edges = np.zeros((num_edges, 2, 4))

    for i in range(num_edges):
        lower = random()
        fake_edges[i] = [[uniform(0, mu_range), uniform(0, std_range), lower, lower],
                         [uniform(0, mu_range), uniform(0, std_range), 1.0 - lower, 1.0]]

    return fake_edges


def plot_histo(data, fig_num):
    plt.clf()
    plt.hist(data, bins=50, color="Purple", density=True)
    plt.show()
    # plt.savefig('{}_costs'.format(fig_num))


class GatherTimes:
    def __init__(self, num_iter, num_edges):
        self.num_iter = num_iter
        self.edges = num_edges
        self.my_times = np.zeros((num_edges, num_iter))
        self.their_times = np.zeros((num_edges, num_iter))
        self.mu_range = 100
        self.std_range = 20
        self.timeit_num = 10

    def generate_data(self):
        tot_time = time()
        for edge in range(self.edges):
            for i in range(self.num_iter):
                fake_edges = generate_edge_data(edge + 2, self.mu_range, self.std_range)
                start = time()
                their_method(fake_edges)
                self.their_times[edge, i] = time() - start

                start = time()
                mcmc(fake_edges)
                self.my_times[edge, i] = time() - start

                if tot_time - time() > 300:
                    self.save_data()
        # self.save_data()

    def save_data(self):
        np.savez('GMM-test-data-{}-100edges'.format(date.today()), my_data=self.my_times, their_data=self.their_times)



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

    # gmm_00 = [[5.0, 0.6, 0.2, 0.2], [10.0, 1.5, 0.8, 1.0]]
    # gmm_01 = [[7.0, 0.4, 0.3, 0.3], [14.0, 1.3, 0.7, 1.0]]
    #
    # gmm_02 = [[1.0, 2.0, 0.7, 0.7], [3.3, 5.0, 0.3, 1.0]]
    # gmm_03 = [[11.2, 0.6, 0.9, 0.9], [33, 1.7, 0.1, 1.0]]
    #
    # gmm_04 = [[60.2, 1.2, 0.5, 0.5], [180.2, 8.3, 0.5, 1.0]]
    # gmm_05 = [[35.1, 4.5, 0.5, 0.5], [17.2, 2.4, 0.5, 1.0]]
    #
    # gmm_06 = [[23.2, 9.6, 0.5, 0.5], [7.5, 0.7, 0.5, 1.0]]
    # gmm_07 = [[23.0, 3.4, 0.5, 0.5], [6.3, 11.0, 0.5, 1.0]]

    # gmms = [gmm_00, gmm_01, gmm_02, gmm_03, gmm_04, gmm_05, gmm_06, gmm_07]
    #
    # gmms = generate_edge_data(3, 100, 20)
    # run_all_methods(gmms, '0_to_7')


    timing_class = GatherTimes(10, 100)
    timing_class.generate_data()
    register(timing_class.save_data)  # at exit

