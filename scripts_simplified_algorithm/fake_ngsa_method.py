#!/usr/bin/env python3

# Python things
import networkx as nx
import numpy as np
import pandas as pd


class NgsaMethod:
    def __init__(self):
        self.df = None
        self.num_paths = 0

    def determine_dominance(self, vals_comp_list):
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

        # print(dom_list)
        new_dom_list = [a - b for [a, b] in dom_list]
        max_indices = [n for n, m in enumerate(new_dom_list) if m == max(new_dom_list)]
        for k in range(len(new_dom_list)):
            vals_list = [self.df.loc[k][vals_comp_list[j][0]] for j in range(len(vals_comp_list))]
            # print(new_dom_list[k], vals_list, self.df.loc[k]['pathName'])

        dom_paths = [self.df.loc[l]['pathName'] for l in max_indices]
        for path in dom_paths:
            print(path)

if __name__ == "__main__":
    ngsa = NgsaMethod()

