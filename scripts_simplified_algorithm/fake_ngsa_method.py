#!/usr/bin/env python3

# Python things
import networkx as nx
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
from topsis import Topsis

class NgsaMethod:
    def __init__(self):
        self.df = None
        self.num_paths = 0
        # self._setup_df()

    def setup_df(self):
        for ind in self.df.index:

            self.df.at[ind, 'ID'] = ind

    def determine_dominance(self, vals_comp_list, return_vals=False):
        self.num_paths = self.df.index.stop
        dom_list = np.zeros((self.num_paths, 2))

        for i in range(self.num_paths - 1):
            path_a = self.df.loc[i]

            for j in range(self.num_paths):
                if j <= i:
                    continue
                path_b = self.df.loc[j]
                a_dom = 0
                b_dom = 0

                for [val_to_comp, min_max] in vals_comp_list:
                    # print(path_a[val_to_comp], path_b[val_to_comp])
                    if min_max == "Min":
                        if path_a[val_to_comp] <= path_b[val_to_comp]:
                            a_dom += 1
                        elif path_b[val_to_comp] <= path_a[val_to_comp]:
                            b_dom += 1
                    elif min_max == "Max":
                        if path_a[val_to_comp] >= path_b[val_to_comp]:
                            a_dom += 1
                        elif path_b[val_to_comp] >= path_a[val_to_comp]:
                            b_dom += 1
                    else:
                        print("Min / max is not a valid entry")

                if a_dom == len(vals_comp_list):
                    dom_list[i][0] += 1
                    dom_list[j][1] += 1
                elif b_dom == len(vals_comp_list):
                    dom_list[j][0] += 1
                    dom_list[i][1] += 1

        print(self.df[[vals_comp_list[0][0], vals_comp_list[1][0]]])
        # print(self.df[])
        print(dom_list)
        # print(dom_list)
        new_dom_list = [a - b for [a, b] in dom_list]
        pareto = [i for i in range(len(new_dom_list)) if dom_list[i][1] == 0]

        scaled_dom_list = []
        min_dom = abs(min(new_dom_list))
        for dom in new_dom_list:
            scaled_dom_list.append(dom + min_dom + 1)

        self.df['dom_num'] = new_dom_list
        self.df['scaled_dom'] = scaled_dom_list

        max_indices = [n for n, m in enumerate(new_dom_list) if m == max(new_dom_list)]
        self.df['color'] = 'red'
        for pareto_index in pareto:
            self.df.at[pareto_index, 'color'] = 'darkgoldenrod'
        for best_index in max_indices:
            self.df.at[best_index, 'color'] = 'blue'
            self.df.at[best_index, 'scaled_dom'] = self.df.loc[best_index]['scaled_dom'] * 3
        dom_paths = [self.df.loc[l]['pathName'] for l in max_indices]
        return dom_paths
        # for path in dom_paths:
        #     print(path)

    def determine_dominance_v2(self, vals_comp_list, return_vals=False):
        self.num_paths = self.df.index.stop
        dom_list = np.zeros((self.num_paths, 2))

        self.df['dom'] = 0
        self.df['dom_by'] = 0

        for i, row in self.df.iterrows():
            path_a = self.df.loc[i]

            for j, _ in self.df.iterrows():
                if j == i:
                    continue
                path_b = self.df.loc[j]
                a_dom = 0
                for [val_to_comp, min_max] in vals_comp_list:
                    # print(path_a[val_to_comp], path_b[val_to_comp])
                    if min_max == "Min":
                        if path_a[val_to_comp] <= path_b[val_to_comp]:
                            a_dom += 1
                    elif min_max == "Max":
                        if path_a[val_to_comp] >= path_b[val_to_comp]:
                            a_dom += 1
                    else:
                        print("Min / max is not a valid entry")

                if a_dom == len(vals_comp_list):
                    self.df.at[i, 'dom'] += 1
                    self.df.at[j, 'dom_by'] += 1

        self.df['dom_num'] = self.df['dom'] - self.df['dom_by']
        # print(self.df[[vals_comp_list[0][0], vals_comp_list[1][0], 'dom', 'dom_by']])

        pareto = np.where(self.df['dom_by'] == 0)[0]

        scaled_dom_list = []
        min_dom = abs(self.df['dom_num'].min()) + 1
        self.df['scaled_dom'] = self.df['dom_num'] + min_dom

        max_val = self.df['dom_num'].max()
        # max_indices = np.where(self.df['dom_num'] == max_val)[0]
        self.df['color'] = 'red'
        self.df['pareto'] = 'no'
        for pareto_index in pareto:
            self.df.at[pareto_index, 'color'] = 'darkgoldenrod'
            self.df.at[pareto_index, 'pareto'] = 'yes'
        dom_paths = self.df[self.df['dom_num'] == max_val]
        # print(dom_paths[[vals_comp_list[0][0], vals_comp_list[1][0]]])
        best_index = dom_paths[[vals_comp_list[0][0]]].idxmin()[0]
        # print(best_index)

        self.df.at[best_index, 'color'] = 'blue'
        self.df.at[best_index, 'scaled_dom'] = self.df.loc[best_index]['scaled_dom'] * 3

        return dom_paths
        # for path in dom_paths:
        #     print(path)

    def use_topsis(self, vals):
        pareto_df = self.df[self.df['pareto'] == 'yes']
        eval_matrix = pareto_df[['ID', vals[0][0], vals[1][0]]].to_numpy(copy=True)
        # print(eval_matrix)
        weights = [0, 1, 1]
        criterion = np.array([False, False, False])
        t = Topsis(eval_matrix, weights, criterion)
        t.calc()
        ranked_list = t.rank_to_best_similarity()
        # best_index = eval_matrix[ranked_list[0]][0]
        print(ranked_list)
        print(eval_matrix)
        # self.df.at[best_index, 'color'] = 'blue'
        # self.df.at[best_index, 'scaled_dom'] = self.df.loc[best_index]['scaled_dom'] * 3
        # print("best_distance\t", t.best_distance)
        # print("worst_distance\t", t.worst_distance)
        #
        # print("worst_similarity\t", t.worst_similarity)
        # print("rank_to_worst_similarity\t", t.rank_to_worst_similarity())
        #
        # print("best_similarity\t", t.best_similarity)
        # print("rank_to_best_similarity\t", t.rank_to_best_similarity())

    def ngsa_graph(self, vals, name):
        x_vals = vals[0][0]
        y_vals = vals[1][0]

        self.df.plot(kind="scatter", x=x_vals, y=y_vals, s='scaled_dom', c=self.df.color)
        plt.title(name)
        plt.savefig('{}_case_03_runoff.svg'.format(name))


if __name__ == "__main__":
    ngsa = NgsaMethod()

