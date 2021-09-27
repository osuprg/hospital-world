#!/usr/bin/env python3

# Python things
import networkx as nx
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt


class NgsaMethod:
    def __init__(self):
        self.df = None
        self.num_paths = 0

    def get_dom_values(self, vals_comp_list):
        pass

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
        print(self.df[[vals_comp_list[0][0], vals_comp_list[1][0], 'dom', 'dom_by']])

        pareto = np.where(self.df['dom_by'] == 0)[0]

        scaled_dom_list = []
        min_dom = abs(self.df['dom_num'].min()) + 1

        self.df['scaled_dom'] = self.df['dom_num'] + min_dom

        max_val = self.df['dom_num'].max()
        max_indices = np.where(self.df['dom_num'] == max_val)[0]
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


    def ngsa_graph(self, vals, name):
        x_vals = vals[0][0]
        y_vals = vals[1][0]

        self.df.plot(kind="scatter", x=x_vals, y=y_vals, s='scaled_dom', c=self.df.color)
        plt.title(name)
        plt.savefig(name)


if __name__ == "__main__":
    ngsa = NgsaMethod()

