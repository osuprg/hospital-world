#!/usr/bin/env python3

import networkx as nx
from network_structure import SimGraph
from dijkstra_varying_weights import run_dijkstra



def path_gmm_distros(t_graph, paths):
    """
    Takes in a NetworkX graph and a list of paths
    Provides GMM distributions for each attribute set and each path
    """

    attr_list = list(list(t_graph.G.edges(data=True))[0][-1].keys())
    num_ranges = len(t_graph.ranges_list)

    for path in paths:
        for i in range(num_ranges):
            float_num_00 = 'fl00_' + '%02d' % i
            float_num_01 = 'fl01_' + '%02d' % i
            std_num_00 = 'std00_' + '%02d' % i
            std_num_01 = 'std01_' + '%02d' % i

        # print(attr[-2:])


if __name__ == "__main__":
    e_list = [(0, 1), (0, 3), (1, 2), (1, 4),
              (3, 2), (3, 4), (2, 5), (4, 5)]
    test_graph = SimGraph(e_list)
    good_paths = run_dijkstra(test_graph)
    path_gmm_distros(test_graph, good_paths)

