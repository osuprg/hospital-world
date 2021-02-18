#!/usr/bin/env python3

# Python things
from time import time
import networkx as nx
from random import random, randint, sample
import numpy as np
from statistics import mean, stdev, StatisticsError
import scipy.stats as stats
import matplotlib.pyplot as plt
import pickle



def plot_gaussians(n1, n2, histo_data, gauss_data, method_name):
    colors = [['lightblue', 'royalblue'],
              ['gold', 'darkgoldenrod'],
              ['coral', 'maroon'],
              ['coral', 'maroon'],
              ['coral', 'maroon'],
              ['purple', 'mediumpurple'],
              ['purple', 'mediumpurple'],
              ['darkgreen', 'limegreen']]

    col_i = 0

    for i in range(len(histo_data)):
        data = histo_data[i][0]
        # print(data)
        name = histo_data[i][1]
        print(name, mean(data), stdev(data), '[', min(data), ', ', max(data), ']')
        plt.hist(data, bins=50, alpha=0.5, color=colors[col_i][0], ec=colors[col_i][0], density=True)
        plt.axvline(mean(data), color=colors[col_i][1], linestyle='dashed', label='mean ' + name)
        plt.axvline(max(data), color=colors[col_i][1], linestyle='dotted', label='max ' + name)
        plt.axvline(min(data), color=colors[col_i][1], linestyle='dotted', label='min' + name)

        col_i += 1

    for j in range(len(gauss_data)):
        data = gauss_data[j][0]
        name = gauss_data[j][1]
        print(name, data[0], data[1], '[', data[2], ', ', data[3], ']')

        mu = data[0]
        std = data[1]
        # low = data[2]
        # high = data[3]
        x = np.linspace(mu - 3 * std, mu + 3 * std, num=1000)
        plt.plot(x, stats.norm.pdf(x, mu, std), c=colors[col_i][0])
        plt.axvline(mu, color=colors[col_i][1], linestyle='dashed', label='mean ' + name)
        # plt.axvline(low, color=colors[col_i][1], linestyle='dotted', label='min ' + name)
        # plt.axvline(high, color=colors[col_i][1], linestyle='dotted', label='max ' + name)
        col_i += 1

    # plt.xlim(x_min, x_max)
    # plt.axis([0, 60, 0, 20])

    plt.title('Path Cost between {} and {}'.format(n1, n2))
    plt.legend(bbox_to_anchor=(1.04, 0), loc="lower left", borderaxespad=0)
    plt.xlabel('Path time (sec)')
    plt.savefig('Path Cost between {} and {} with GMM, {}'.format(n1, n2, method_name))
    plt.clf()
