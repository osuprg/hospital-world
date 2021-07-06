#!/usr/bin/env python3

import networkx as nx
from network_structure import SimGraph


def run_dijkstra(t_graph):
    # Gets the list of edge attributes
    attr_list = list(list(t_graph.G.edges(data=True))[0][-1].keys())
    path_list = []

    # Plans a path in Dijkstra for each edge weight
    for attr in attr_list:
        path = nx.dijkstra_path(t_graph.G, 0, 5, weight=attr)
        if path not in path_list:
            path_list.append(path)

    return path_list


if __name__ == "__main__":
    e_list = [(0, 1), (0, 3), (1, 2), (1, 4),
              (3, 2), (3, 4), (2, 5), (4, 5)]
    test_graph = SimGraph(e_list)
    good_paths = run_dijkstra(test_graph)
    print(good_paths)
