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

# Custom things
import POST_compare_choice_methods as Choices
import STRUCT_hospital_graph_class as HospGraph
import RUN_custom_global_plan_algebraic as Alg
import RUN_custom_global_plan_sampling as Samp
import RUN_custom_global_plan_all_paths as All
import POST_plot_stacked_gaussians as PltGaus

# For reference to column names
# self.df = pd.DataFrame({'path': [], 'methods': [], 'pathName': [], 'methodNames': [],
#                         'sampleData': [], 'gmmFit': [],
#                         'humDist': [], 'fourConnDist': [],
#                         'gausMean': [], 'gausStd': [],
#                         'gmmData': []})

class ManipulateDF:
    def __init__(self, df):
        self.df = df

    def manipulate_df(self):
        # single_val = ['humDist', 'fourConnDist', 'gausMean', 'gausStd']
        # for val in single_val:
        #     self.add_scaled_val(val)

        self.mess_with_gmm()

    def add_scaled_val(self, val):
        max_val = self.df.max(0)[val]
        val_a = val + 'A'
        val_scaled = val + 'Scaled'
        print("%%%%%%%%%%%%")
        print('scaling:')
        print(self.df[val])
        self.df[val_a] = self.df.apply(lambda x: (max_val - x[val]) / max_val, axis=1)
        self.df[val_scaled] = self.df.apply(lambda y: (y[val_a] / self.df.sum(0)[val_a]), axis=1)
        print('scaled:')
        print(self.df[val_scaled])

    def mess_with_gmm(self):
        for j in range(len(self.df['gmmData'][0])):
            gmm_mean = 'gmmMean' + str(j)
            gmm_std = 'gmmSTD' + str(j)
            gmm_weight = 'gmmWeight' + str(j)
            self.df[gmm_mean] = 1000.0
            self.df[gmm_std] = 1000.0
            self.df[gmm_weight] = 1000.0

        for index, row in self.df.iterrows():
            gmm_num = 0
            # print(index, row)

            for [mean, std, weight] in self.df['gmmData'][index]:
                gmm_mean = 'gmmMean' + str(gmm_num)
                gmm_std = 'gmmSTD' + str(gmm_num)
                gmm_weight = 'gmmWeight' + str(gmm_num)

                self.df.loc[index, gmm_mean] = mean
                self.df.loc[index, gmm_std] = std
                self.df.loc[index, gmm_weight] = weight
                #
                # print(self.df[gmm_mean])
                # print(self.df[gmm_std])
                # print(self.df[gmm_weight])

                gmm_num += 1
                # print('==========')

        for i in range(gmm_num):
            gmm_mean = 'gmmMean' + str(i)
            gmm_std = 'gmmSTD' + str(i)
            self.add_scaled_val(gmm_mean)
            self.add_scaled_val(gmm_std)



if __name__ == "__main__":

    path_to_pickle = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/hospital_trials_2021-02-11_hum_50_70_plus_stats_clean'
    path_to_paths_file = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/paths_generated_2021-02-11_clean_run_01'
    hosp_graph = HospGraph.unpickle_it(path_to_pickle)
    sampling_planner = Samp.SamplingPlannerClass(hosp_graph)

    # data_accum = Samp.DataAccumulated()
    # all_paths_planner = All.AllPathsPlanner(hosp_graph)

    iterations = 1000
    sample_size = 1000

    n1 = 'r00'
    n2 = 'r10'

    compare = Choices.CompareMethods(hosp_graph, iterations)

    compare.main_loop(n1, n2)
    manipulate = ManipulateDF(compare.df)
    manipulate.manipulate_df()

