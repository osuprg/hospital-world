#!/usr/bin/env python3

import STRUCT_hospital_graph_class as HospGraph
from networkx import dijkstra_path

#TODO: The statistics should be calculated in the file that collects / saves data
import statistics

# Yes this is terribly dumb, but you have to write out the full filepath
# Otherwise it will save to the directory from which you are running the script
path_and_name = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/hospital_trials_2021-01-15_plus_stats_clean'


def fallback(dict):
    euclid = dict['euclidean_dist']
    # return euclid / 0.22
    return 1000


def mean_one_gaussian(n1, n2, dict):
    """ Ignores human condition. Returns the mean of all data on the edge.
    If there is no data, returns the euclidean distance divided by the top speed"""

    mean_all = dict['mean_all']

    if mean_all:
        return mean_all
    else:
        return fallback(dict)


def weighted_95_percentile(n1, n2, dict):
    pct_hum = dict['pct_hum']
    mean_hum = dict['mean_hum']
    std_hum = dict['std_hum']
    mean_no = dict['mean_no']
    std_no = dict['std_no']


    # If there is nothing for mean_hum
    if not mean_hum:
        # and nothing for mean_no
        if not mean_no:
            # Return the euclidean distance, divided by 80% top speed (for time), plus two standard deviations
            return fallback(dict) + (2 * 2.718)  # 2.718 is the average standard deviation across all edges
        # Otherwise return mean_no
        else:
            return mean_no + (2 * std_no)
    # if there is nothing in mean_no, return mean_hum
    elif not std_no:
        return mean_hum + (2 * std_hum)
    # Otherwise use the weighted sum of their means
    else:
        return pct_hum * (mean_hum + (2 * std_hum)) + (1 - pct_hum) * (mean_no + (2 * std_no))


def hum_95_percentile(n1, n2, dict):
    pct_hum = dict['pct_hum']
    mean_hum = dict['mean_hum']
    std_hum = dict['std_hum']
    mean_no = dict['mean_no']
    std_no = dict['std_no']

    if mean_hum:
        return mean_hum + 2 * std_hum
    elif mean_no:
        return mean_no + 2 * std_no
    else:
        return fallback(dict)

def no_hum_95_percentile(n1, n2, dict):
    pct_hum = dict['pct_hum']
    mean_hum = dict['mean_hum']
    std_hum = dict['std_hum']
    mean_no = dict['mean_no']
    std_no = dict['std_no']

    if mean_no:
        return mean_no + 2 * std_no
    else:
        return fallback(dict)

def mean_no_hum(n1, n2, dict):
    mean_no = dict['mean_no']
    if mean_no:
        return mean_no
    else:
        return fallback(dict)

def std_and_hum(n1, n2, dict):
    pct_hum = dict['pct_hum']
    std_no = dict['std_no']
    std_hum = dict['std_hum']

    if std_hum:
        return pct_hum + (std_hum * pct_hum) + ((1 - pct_hum) * std_no)
    else:
        return std_no

def four_conn_hum(n1, n2, dict):
    four_conn = dict['four_connect_dist']
    pct_hum = dict['pct_hum']
    return four_conn * pct_hum

def num_doors(n1, n2, dict):
    cost = 0
    if 'd' in n1:
        cost += 0.5
    if 'd' in n2:
        cost += 0.5

    return cost


if __name__ == '__main__':
    hosp_graph = HospGraph.unpickle_it(path_and_name)

    """
    Dijkstra can accept function calls as a weight. From documentation:
    If this is a function, the weight of an edge is the value returned by the function.
    The function must accept exactly three positional arguments:
    the two endpoints of an edge and the dictionary of edge attributes for that edge.
    The function must return a number.
    """
    diff_euc = 0
    diff_four = 0
    num = 0
    # for (n1, n2) in hosp_graph.edges():
    #     print(std_and_hum(n1, n2, hosp_graph[n1][n2]))
        # print(mean_no_hum(n1, n2, hosp_graph[n1][n2]))
    print(dijkstra_path(hosp_graph, 'r00', 'r08', weight=std_and_hum))
    # print(dijkstra_path(hosp_graph, 'r00', 'r08', weight=mean_one_gaussian))
    # print(dijkstra_path(hosp_graph, 'r00', 'r08', weight=weighted_95_percentile))
