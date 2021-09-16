#!/usr/bin/env python3

import STRUCT_hospital_graph_class as HospGraph
from networkx.algorithms.simple_paths import shortest_simple_paths
from itertools import islice
import RUN_custom_global_plan_sampling as Samp
import POST_compare_choice_methods as Choices
import numpy as np
from sklearn import mixture
from scripts_simplified_algorithm.cow_goes_MOO import ManipulateDF

path_and_name = '/home/anna/workspaces/research_ws/src/hospital_world/pickles/hospital_trials_2021-01-15_plus_stats_clean'


def try_GMM(costs_gaus):
    # costs_dummy = np.array(costs_gaus)
    costs_dummy = np.reshape(costs_gaus, (-1, 1))
    # costs_gaus_t = np.ndarray.transpose(costs_dummy)
    bayes_gmm = mixture.BayesianGaussianMixture(n_components=2)
    bayes_data = bayes_gmm.fit(costs_dummy)
    return bayes_data

if __name__ == '__main__':
    hosp_graph = HospGraph.unpickle_it(path_and_name)

    shortest_simple = [i for i in islice(shortest_simple_paths(hosp_graph, 'r00', target='r10', weight='mean_all'), 10)]

    methods = []
    for j in range(len(shortest_simple)):
        methods.append(Choices.MethodDef('method{}'.format(j), None))
    shortest_reformat = [[[methods[i]], shortest_simple[i]] for i in range(len(shortest_simple))]
    sampling_planner = Samp.SamplingPlannerClass(hosp_graph)

    iterations = 1000
    sample_size = 1000

    n1 = 'r00'
    n2 = 'r10'

    compare = Choices.CompareMethods(hosp_graph, iterations)

    compare.compare_paths_v2(shortest_reformat)
    manipulate = ManipulateDF(compare.df)
    manipulate.manipulate_df()




