#!/usr/bin/env python3

import pickle
import STRUCT_hospital_graph_class as HospGraph


class PathsData:
    def __init__(self, paths_data, hosp_graph):
        self.paths = paths_data.paths
        self.timings = paths_data.times
        self.hosp_graph = hosp_graph
        # Custom then dijkstra path count for (0, 1, 2) condition
        self.stats = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.tot_hum_cond = [0, 0, 0]
        self.diff_cust_dij = 0
        self.diff_alg_dij = 0
        self.diff_cust_alg = 0
        self.tot = 0  # Because I'm lazy

        self.add_nums_to_edges()
        # self.count_hum_cond()

    def same_or_diff(self, custom, algebraic, dijkstra):
        diff = [i for i in dijkstra if i not in custom]
        diff2 = [j for j in custom if j not in dijkstra]

        diff3 = [k for k in algebraic if k not in dijkstra]
        diff4 = [l for l in dijkstra if l not in algebraic]

        diff5 = [m for m in custom if m not in algebraic]
        diff6 = [n for n in algebraic if n not in custom]

        if len(diff) > 1 and len(diff2) > 1:
            self.diff_cust_dij += 1
            # print('custom and dijkstra are different')
            # print('cust: {}'.format(custom))
            # print('dijk: {}'.format(dijkstra))
            # print(diff, diff2)

        if len(diff3) > 1 and len(diff4) > 1:
            self.diff_alg_dij += 1
            # print('algebraic and dijkstra are different')
            # print('alg: {}'.format(algebraic))
            # print('dijk: {}'.format(dijkstra))
            # print(diff3, diff4)

        if len(diff5) > 1 and len(diff6) > 1:
            self.diff_cust_alg += 1
            # print('custom and algebraic are different')
            # print('cust: {}'.format(custom))
            # print('alg: {}'.format(algebraic))
            # print(diff5, diff6)

    def add_nums_to_edges(self):
        # paths_dummy = self.paths[0:3]
        # print(paths_dummy)
        for [custom, algebraic, dijkstra] in self.paths:
            # print('-------custom----------')
            self.same_or_diff(custom, algebraic, dijkstra)
            self.tot += 1

            for i in range(len(custom) - 1):
                n1 = custom[i]
                n2 = custom[i + 1]
                hum_cond = int(self.hosp_graph[n1][n2]['hum_cond'])
                # print(n1, n2, hum_cond)
                self.stats[0][hum_cond] += 1
                # print(self.stats)

            for j in range(len(algebraic) - 1):
                n1 = algebraic[j]
                n2 = algebraic[j + 1]
                hum_cond = int(self.hosp_graph[n1][n2]['hum_cond'])
                # print(n1, n2, hum_cond)
                self.stats[1][hum_cond] += 1
                # print(self.stats)

            # print('-------dijkstra----------')
            for k in range(len(dijkstra) - 1):
                n1 = dijkstra[k]
                n2 = dijkstra[k + 1]
                hum_cond = int(self.hosp_graph[n1][n2]['hum_cond'])
                # print(n1, n2, hum_cond)
                self.stats[2][hum_cond] += 1
                # print(self.stats)

        print('custom {}'.format(self.stats[0]))
        print('algebraic {}'.format(self.stats[1]))
        print('dijkstra {}'.format(self.stats[2]))
        print('custom vs dijkstra: {}'.format((1 - self.diff_cust_dij / self.tot) * 100))
        print('algebraic vs dijkstra: {}'.format((1 - self.diff_alg_dij / self.tot) * 100))
        print('custom vs algebraic: {}'.format((1 - self.diff_cust_alg / self.tot) * 100))

    def count_hum_cond(self):
        for (n1, n2) in self.hosp_graph.edges():
            self.tot_hum_cond[int(self.hosp_graph[n1][n2]['hum_cond'])] += 1

        print(self.tot_hum_cond)


def unpickle_it(filename):
    infile = open(filename, 'rb')
    unpickled = pickle.load(infile)
    infile.close()
    return unpickled

class DataAccumulated:
    def __init__(self):
        self.paths = []
        self.times = []

if __name__ == "__main__":
    path_to_pickle = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/hospital_trials_2021-01-15_plus_stats_clean'
    path_to_paths_file = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/paths_generated_2021-01-15_clean_data_run01'
    paths_data = unpickle_it(path_to_paths_file)
    hosp_graph = HospGraph.unpickle_it(path_to_pickle)

    data_analysis = PathsData(paths_data, hosp_graph)
