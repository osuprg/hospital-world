#!/usr/bin/env python3

import networkx as nx
from sklearn import mixture
from random import random, gauss
import numpy as np
import matplotlib.pyplot as plt


def sample_data(gmm_list):
    samples = []
    for _ in range(10000):
        rand_num = random()
        for [mu, sigma, wt, add_wt] in gmm_list:
            if rand_num < add_wt:
                samples.append(gauss(mu, sigma))
                break

    return samples


def plot_it(gmm_list, name=None):

    plt.clf()
    data = sample_data(gmm_list)
    plt.hist(data, bins=50, density=True)
    plt.title(name)
    plt.savefig(name)


if __name__ == "__main__":
    g0 = [13.2, 0.7211102550927979, 0.4, 0.4]
    g1 = [33.2, 1.4317821063276355, 0.4, 0.8]
    g2 = [119.5, 1.746424919657298, 0.1, 0.9]
    g3 = [139.5, 2.1400934559032696, 0.1, 1.0]

    g4 = [23.04016129907458, 10.070536283214105, 0.7926207188334234, 0.7926207188334234]
    g5 = [129.68955089101965, 10.343765350700854, 0.20737928116657664, 1]

    set00 = [g0, g1, g2, g3]
    set01 = [g4, g5]

    plot_it(set00)
    plot_it(set01)
