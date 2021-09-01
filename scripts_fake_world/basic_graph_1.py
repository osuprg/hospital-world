#!/usr/bin/env python2

import networkx as nx
from networkx.drawing.nx_agraph import graphviz_layout
import matplotlib.pyplot as plt
import numpy as np
from random import uniform
import pickle
from datetime import date

class Graph1:
    def __init__(self, num_nodes, edge_links):
        # Max 100 nodes

        self.G = nx.Graph()
        self.num_nodes = num_nodes
        self.edge_links = edge_links
        self.nodes = []

        self.range_pairs = [(0, 1), (1, 10), (10, 100), (100, 500), (500, 1000), (1000, 5000)]
        self.set_up_nodes()
        self.set_up_edges()
        self.plot_graph()

    def set_up_nodes(self):
        self.nodes = ['n' + '%02d' % i for i in range(self.num_nodes)]
        self.G.add_nodes_from(self.nodes)

    def set_up_edges(self):
        self.G.add_edges_from(self.edge_links)
        for (n1, n2) in self.G.edges():
            for (r1, r2) in self.range_pairs:
                sc_name = 'sc_' + str(r1) + '_' + str(r2)
                mean_a_name = 'mu_a_' + str(r1) + '_' + str(r2)
                stdev_a_name = 'std_a_' + str(r1) + '_' + str(r2)
                mean_b_name = 'mu_b_' + str(r1) + '_' + str(r2)
                stdev_b_name = 'std_b_' + str(r1) + '_' + str(r2)

                self.G[n1][n2][sc_name] = uniform(r1, r2)
                self.G[n1][n2][mean_a_name] = uniform(r1, r2)
                self.G[n1][n2][mean_b_name] = uniform(r1, r2)

                # Standard deviation will be no more than 30% of the range
                stdev_range = (r2 - r1) * 0.3
                self.G[n1][n2][stdev_a_name] = uniform(0, stdev_range)
                self.G[n1][n2][stdev_b_name] = uniform(0, stdev_range)

            print(self.G[n1][n2])

    def plot_graph(self):
        # Plot the graph
        pos = graphviz_layout(self.G, prog='dot')
        nx.draw_networkx(self.G, pos, with_labels=True)

        plt.show()

if __name__ == "__main__":
    num_nodes1 = 4
    edge_links1 = [('n00', 'n01'), ('n00', 'n02'), ('n01', 'n03'), ('n02', 'n03')]
    graph1 = Graph1(num_nodes1, edge_links1)
