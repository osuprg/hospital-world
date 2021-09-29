#!/usr/bin/env python3

# Python things
import pandas as pd

# Custom things
import compare_paths as Choices
import fake_hospital_graph as HospGraph
import sampling_planner as Samp


class ManipulateDF:
    def __init__(self):
        self.df = pd.DataFrame()
        self.results = pd.DataFrame()

    def setup_df_vals(self):
        single_val = ['humDist', 'fourConnDist', 'gausMean', 'gausStd', 'doors', 'navFail']

        # Scales values to
        # for val in single_val:
        #     self._add_scaled_val(val)

        self._scale_gmm_data()

    def _add_cols_to_df(self):
        for dij_fun in self.df['methodNames'][0]:
            pass

    def scale_min(self, x, min_val):
        if x == 0:
            x = 0.01
        return min_val / x

    def scale_max(self, x, max_val):
        if x == 0:
            x = 0.001
        return x / max_val

    def _add_scaled_val(self, val):
        """
        Scales the given value to [0,1], inverse of the value.
        Low values get a higher scaled value, which will determine how many votes it will get.
        We want to reward low values
        """

        val_min_scaled = val + 'ScaledMin'
        val_max_scaled = val + 'ScaledMax'
        new_arr = []
        for x in self.df[val]:
            if x <= 0:
                x = 0.0001
            new_arr.append(x)

        self.df[val] = new_arr

        max_val = self.df.max(0)[val] * 1.10
        min_val = self.df.min(0)[val] * 0.9

        if max_val == 0:
            max_val = 0.01
        if min_val == 0:
            min_val = 0.01

        max_min_diff = max_val - min_val
        # If the numbers are all the same
        if max_min_diff == 0:
            max_min_diff = 0.01
        # My method
        # self.df[val_max_a] = self.df.apply(lambda a: self.scale_max(a[val], max_val), axis=1)
        # self.df[val_max_scaled] = self.df.apply(lambda c: (c[val_max_a] / self.df.sum(0)[val_max_a]), axis=1)
        # self.df[val_min_a] = self.df.apply(lambda x: self.scale_min(x[val], min_val), axis=1)
        # self.df[val_min_scaled] = self.df.apply(lambda z: (z[val_min_a] / self.df.sum(0)[val_min_a]), axis=1)

        # SAW method
        self.df[val_max_scaled] = self.df.apply(lambda a: (a[val] - min_val) / (max_min_diff), axis=1)
        # sum_val_max_a = self.df.sum(0)[val_max_a]
        # if sum_val_max_a == 0:
        #     sum_val_max_a = 0.01
        # self.df[val_max_scaled] = self.df.apply(lambda c: (c[val_max_a] / sum_val_max_a), axis=1)

        self.df[val_min_scaled] = self.df.apply(lambda x: (max_val - x[val]) / (max_min_diff), axis=1)
        # sum_val_min_a = self.df.sum(0)[val_min_a]
        # if sum_val_min_a == 0:
        #     sum_val_min_a = 0.01
        # self.df[val_min_scaled] = self.df.apply(lambda z: (z[val_min_a] / sum_val_min_a), axis=1)

        # self.df.drop([val_min_a, val_max_a], axis=1)

    def _scale_gmm_data(self):
        """
        Unpacks GMM data to individual columns then scales the mean and stdev
        """
        for j in range(len(self.df['gmmData'][0])):
            gmm_mean = 'gmmMean' + str(j)
            gmm_std = 'gmmSTD' + str(j)
            gmm_weight = 'gmmWeight' + str(j)
            gmm_upper = 'gmmUpper' + str(j)
            gmm_lower = 'gmmLower' + str(j)
            self.df[gmm_mean] = 1000.0
            self.df[gmm_std] = 1000.0
            self.df[gmm_weight] = 1000.0
            self.df[gmm_upper] = 1000.0
            self.df[gmm_lower] = 1000.0

        for k, row in self.df.iterrows():
            gmm_num = 0

            for [mean, std, weight] in self.df['gmmData'][k]:
                gmm_mean = 'gmmMean' + str(gmm_num)
                gmm_std = 'gmmSTD' + str(gmm_num)
                gmm_weight = 'gmmWeight' + str(gmm_num)
                gmm_upper = 'gmmUpper' + str(gmm_num)
                gmm_lower = 'gmmLower' + str(gmm_num)

                self.df.loc[k, gmm_mean] = mean
                self.df.loc[k, gmm_std] = std
                self.df.loc[k, gmm_weight] = weight
                self.df.loc[k, gmm_upper] = mean + 2 * std
                self.df.loc[k, gmm_lower] = mean - 2 * std

                gmm_num += 1

        for i in range(gmm_num):
            gmm_mean = 'gmmMean' + str(i)
            gmm_std = 'gmmSTD' + str(i)
            gmm_upper = 'gmmUpper' + str(i)
            gmm_lower = 'gmmLower' + str(i)
            self._add_scaled_val(gmm_mean)
            self._add_scaled_val(gmm_std)
            self._add_scaled_val(gmm_upper)
            self._add_scaled_val(gmm_lower)

    def get_dominant_path(self, votes_arr):
        self.df['votes'] = 0
        for [cost, minmax] in votes_arr: # votes
            cost_name = cost + 'Scaled' + minmax
            self.df['votes'] += self.df[cost_name]  # * votes
        winner_i = self.df['votes'].idxmax()
        winner_path = self.df['pathName'][winner_i]
        return winner_path


if __name__ == "__main__":

    path_to_pickle = '/pickles/hospital_trials_2021-02-11_hum_50_70_plus_stats_clean'
    path_to_paths_file = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/paths_generated_2021-02-11_clean_run_01'
    hosp_graph = HospGraph.unpickle_it(path_to_pickle)
    sampling_planner = Samp.SamplingPlannerClass(hosp_graph)

    # data_accum = Samp.DataAccumulated()
    # all_paths_planner = All.AllPathsPlanner(hosp_graph)

    iterations = 1000
    sample_size = 1000

    compare = Choices.CompareMethods(hosp_graph, iterations)

    means_arr = [['gmmUpper1', 'Min', 10], ['gausMean', 'Min', 5]]
    min_hum_arr = [['doors', 'Min', 5], ['humDist', 'Min', 10]]
    max_hum_arr = [['humDist', 'Max', 10], ['gmmMean1', 'Min', 3]]
    evening_arr = [['fourConnDist', 'Min', 5], ['gmmUpper0', 'Min', 10]]
    go_fast = [['gmmUpper0', 'Min', 10], ['gmmUpper1', 'Min', 3]]

    goals = ['Guaranteed not slow', 'Delicate', 'See people', 'Go fast after hours', 'GOFAST']
    methods = [means_arr, min_hum_arr, max_hum_arr, evening_arr, go_fast]
    manipulate = ManipulateDF()

    for rm1 in range(21):
        for rm2 in range(21):

            if rm2 <= rm1:
                continue

            n1 = 'r' + "{:02d}".format(rm1)
            n2 = 'r' + "{:02d}".format(rm2)

            mult_paths = compare.main_loop(n1, n2)
            manipulate.df = compare.df
            num_compare = 0

            if mult_paths:

                manipulate.setup()

                for i in range(len(methods)):
                    method = methods[i]
                    # print(goals[i])
                    manipulate.get_dominant_path(method)

                num_compare += 1


# For reference to column names
# self.df = pd.DataFrame({'path': [], 'methods': [], 'pathName': [], 'methodNames': [],
#                         'sampleData': [], 'gmmFit': [],
#                         'humDist': [], 'fourConnDist': [], 'doors': [],
#                         'gausMean': [], 'gausStd': [],
#                         'gmmData': [], 'navFail': []})
