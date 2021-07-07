#!/usr/bin/env python3

import networkx as nx
import pickle
from random import uniform


class SimGraph:
    def __init__(self, edge_list):
        self.G = nx.Graph()
        self.G.add_edges_from(edge_list)
        self.ranges_list = [(0, 1), (1, 10), (10, 100), (100, 1000)]
        self.add_edge_weights()

    def add_edge_weights(self):
        for (n1, n2) in self.G.edges():
            range_num = 0
            for i in range(len(self.ranges_list)):
                (low, high) = self.ranges_list[i]
                # Add two float values in each range to each edge. Can be used as floats or means for GMM
                float_num_00 = 'fl00_' + '%02d' % range_num
                float_num_01 = 'fl01_' + '%02d' % range_num
                self.G[n1][n2][float_num_00] = uniform(low, high)
                self.G[n1][n2][float_num_01] = uniform(low, high)

                std_num_00 = 'std00_' + '%02d' % range_num
                std_num_01 = 'std01_' + '%02d' % range_num
                if i == 0:
                    # Add two float values in each range to each edge. Can be used as floats or standard devs for GMM
                    self.G[n1][n2][std_num_00] = uniform(low, high)
                    self.G[n1][n2][std_num_01] = uniform(low, high)
                else:
                    low = 1
                    high = 15
                    self.G[n1][n2][std_num_00] = uniform(low, high)
                    self.G[n1][n2][std_num_01] = uniform(low, high)

                range_num += 1


def pickle_it(obj_to_pickle, file_path_and_name):
    with open(file_path_and_name, 'wb') as f:
        pickle.dump(obj_to_pickle, f)
    print("Saving information to file named {}".format(file_path_and_name))


def unpickle_it(filename):
    infile = open(filename, 'rb')
    unpickled = pickle.load(infile)
    infile.close()
    return unpickled


if __name__ == "__main__":
    e_list = [(0, 1), (0, 2), (1, 3), (2, 3)]

    test_graph = SimGraph(e_list)

    for (n1, n2) in test_graph.G.edges():
        print(test_graph.G[n1][n2])

