#!/usr/bin/env python3

import networkx as nx
import pickle
from random import uniform, randint, random
from math import sqrt
from networkx.drawing.nx_agraph import graphviz_layout
import matplotlib.pyplot as plt


class SimGraph:
    def __init__(self, edge_list):
        self.G = nx.Graph()
        self.G.add_edges_from(edge_list)
        self.edge_length_range = [2, 10]  # meters
        self.robot_max_speed = 1  # m / s
        self.robot_avg_speed = self.robot_max_speed * 0.9
        self.std_range = [2, 10]
        self.gauss_range = [0.9, 1.1]
        self.edge_conditions = [[1.0, 1.0], [0.4, 0.75], [0.8, 0.5]]
        self.prob_nav_fail = 0.05
        self.add_edge_weights()

    def gauss_noise(self):
        # Because it was annoying and messy to write this over and over again
        return uniform(self.gauss_range[0], self.gauss_range[1])

    def add_edge_weights(self):
        for (n1, n2) in self.G.edges():
            # Determine edge length
            self.G[n1][n2]['euclidean_dist'] = uniform(self.edge_length_range[0], self.edge_length_range[1])
            self.G[n1][n2]['sq_dist'] = self.G[n1][n2]['euclidean_dist']

            # Determine edge condition
            edge_condition = randint(0, 2)
            self.G[n1][n2]['edge_cond'] = edge_condition

            # Determine lower Gaussian mean - square distance divided by the average speed with uniform noise
            mean_no = self.G[n1][n2]['sq_dist'] / (self.robot_avg_speed * self.gauss_noise())
            std_no = uniform(self.std_range[0], self.std_range[1])
            self.G[n1][n2]['mean_no'] = mean_no
            self.G[n1][n2]['std_no'] = std_no
            self.G[n1][n2]['95_no'] = self.G[n1][n2]['mean_no'] + 2 * self.G[n1][n2]['std_no']
            self.G[n1][n2]['nav_fail'] = 0
            self.G[n1][n2]['doors'] = randint(0, 2)

            if random() < self.prob_nav_fail:
                self.G[n1][n2]['nav_fail'] = randint(1, 3)

            # There are no humans on this edge
            if edge_condition == 0:
                self.G[n1][n2]['mean_hum'] = None
                self.G[n1][n2]['std_hum'] = None
                self.G[n1][n2]['95_hum'] = None
                self.G[n1][n2]['pct_hum'] = 0.0
                self.G[n1][n2]['mean_all'] = mean_no
                self.G[n1][n2]['std_all'] = std_no
                self.G[n1][n2]['95_all'] = self.G[n1][n2]['95_no']

            else:
                # Scale the average speed according to the edge condition and add uniform noise
                mean_hum =self.G[n1][n2]['sq_dist'] / (self.robot_avg_speed *
                                                       self.edge_conditions[edge_condition][1] *
                                                       self.gauss_noise())
                std_hum = uniform(self.std_range[0], self.std_range[1])
                pct_hum = self.edge_conditions[edge_condition][0] * self.gauss_noise()
                self.G[n1][n2]['mean_hum'] = mean_hum
                self.G[n1][n2]['std_hum'] = std_hum
                self.G[n1][n2]['pct_hum'] = pct_hum
                self.G[n1][n2]['95_hum'] = self.G[n1][n2]['mean_hum'] + 2 * self.G[n1][n2]['std_hum']

                mean_all = (mean_hum * pct_hum) + mean_no * (1 - pct_hum)
                std_all = sqrt(std_no ** 2 + std_hum ** 2)
                upper_95_all = mean_all + 2 * std_all
                self.G[n1][n2]['mean_all'] = mean_all
                self.G[n1][n2]['std_all'] = std_all
                self.G[n1][n2]['95_all'] = upper_95_all

            self.G[n1][n2]['hum_dist'] = self.G[n1][n2]['pct_hum'] * self.G[n1][n2]['sq_dist']

    def plot_graph(self):
        # Plot the graph
        pos = graphviz_layout(self.G, prog='dot')
        nx.draw_networkx(self.G, pos, with_labels=True)

        plt.show()



def pickle_it(obj_to_pickle, file_path_and_name):
    with open(file_path_and_name, 'wb') as f:
        pickle.dump(obj_to_pickle, f)
    print("Saving information to file named {}".format(file_path_and_name))


def unpickle_it(filename):
    infile = open(filename, 'rb')
    unpickled = pickle.load(infile)
    infile.close()
    return unpickled


def create_edge_list(num_edges, loop_every):
    if num_edges % loop_every:
        raise ValueError("num_edges must be divisible by loop_every")

    edge_list = []
    rows = num_edges / loop_every
    for edge in range(num_edges):
        if edge % loop_every == loop_every - 1:
            pass
        else:
            edge_list.append((edge, edge + 1))

        if num_edges - edge < loop_every + 1:
            pass
        else:
            edge_list.append((edge, edge + loop_every))

    return edge_list

if __name__ == "__main__":
    # e_list = [(0, 1), (0, 2), (1, 3), (2, 3)]

    num_nodes = 100
    loop = 10
    edge_list = create_edge_list(num_nodes, loop)
    test_graph = SimGraph(edge_list)
    test_graph.plot_graph()

    pickle_it(test_graph.G, 'fake_hospital_graph_{}_nodes'.format(num_nodes))
    #
    # for (n1, n2) in test_graph.G.edges():
    #     print(test_graph.G[n1][n2])

