#!/usr/bin/env python3

import pickle
from matplotlib import pyplot as plt
from statistics import mean
from math import sqrt
import RUN_custom_global_plan_sampling as Samp
import STRUCT_hospital_graph_class as HospGraph
import numpy as np
from sklearn import mixture


def unpickle_it(filename):
    infile = open(filename, 'rb')
    unpickled = pickle.load(infile)
    infile.close()
    return unpickled


path_to_pickle = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/hospital_trials_2021-02-11_hum_50_70_plus_stats_clean'
hosp_graph = HospGraph.unpickle_it(path_to_pickle)
file_to_open = "/home/toothless/workspaces/research_ws/src/hospital-world/pickles/path_data_from_move_base-02-24-2021"

sampling_planner = Samp.SamplingPlannerClass(hosp_graph)
path = [['nothing', ['r00', 'r00_d00', 'r01_d00', 'r01_d01', 'h01', 'h02', 'r11_d00', 'r10_d00', 'r10']]]
path_data_sampling, _ =sampling_planner.run_backward_v2(path, 1000, real_data=False)

path_data_ros = unpickle_it(file_to_open)

samp_costs_dummy = np.reshape(path_data_sampling, (-1, 1))
bayes_gmm = mixture.BayesianGaussianMixture(n_components=2)
samp_costs_gmm = bayes_gmm.fit(samp_costs_dummy)

ros_costs_dummy = np.reshape(path_data_ros, (-1, 1))
bayes_gmm_ros = mixture.BayesianGaussianMixture(n_components=2)
ros_costs_gmm = bayes_gmm_ros.fit(ros_costs_dummy)

print('sampling planner gmm')
for i in range(len(samp_costs_gmm.means_)):
    print(samp_costs_gmm.means_[i], sqrt(samp_costs_gmm.covariances_[i]), samp_costs_gmm.weights_[i])

print('ros gmm')
for i in range(len(ros_costs_gmm.means_)):
    print(ros_costs_gmm.means_[i], sqrt(ros_costs_gmm.covariances_[i]), ros_costs_gmm.weights_[i])

plt.hist(path_data_ros, bins=50, alpha=0.6, color='royalblue', density=True, label="ROS")
plt.hist(path_data_sampling, bins=50, alpha=0.5, color='orange', density=True, label="Test Environment")
plt.legend(loc="upper left")
plt.title('Comparison of data from test environment and ROS')
plt.xlabel('Seconds to complete path')
plt.savefig('testvmovebase.svg', dpi=500)
