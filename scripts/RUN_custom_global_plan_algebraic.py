#!/usr/bin/env python3

import STRUCT_hospital_graph_class as HospGraph
from networkx import dijkstra_path

#TODO: The statistics should be calculated in the file that collects / saves data
import statistics

# Yes this is terribly dumb, but you have to write out the full filepath
# Otherwise it will save to the directory from which you are running the script
path_and_name = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/hospital_trials_2021-01-15_plus_stats_clean'


def mean_one_gaussian(n1, n2, dict):
    """ Ignores human condition. Returns the mean of all data on the edge.
    If there is no data, returns the euclidean distance divided by the top speed"""

    data_hum = dict['trav_data_hum']
    data_no = dict['trav_data_no']
    data_all = data_hum + data_no
    euclidean = dict['euclidean_dist']

    if data_all:
        mean_all = statistics.mean(data_all)
        return mean_all
    else:
        return euclidean / (0.22 * 0.8)


def weighted_means(n1, n2, dict):
    """
    Returns the weighted mean of data on the edge.
    Exceptions for when there is no data in one or both arrays
    """
    pct_hum = dict['pct_hum']
    mean_hum = dict['mean_hum']
    mean_no = dict['mean_no']
    euclidean = dict['euclidean_dist']

    # If there is nothing for mean_hum
    if not mean_hum:
        # and nothing for mean_no
        if not mean_no:
            # Divide the euclidean distance by the top speed of the robot, dampened by 20%
            return euclidean / (0.22 * 0.8)

        # Otherwise return mean_no
        else:
            return mean_no
    # if there is nothing in mean_no, return mean_hum
    elif not mean_no:
        return mean_hum
    # Otherwise use the weighted sum of their means
    else:
        return pct_hum * mean_hum + (1 - pct_hum) * mean_no


def weighted_95_percentile(n1, n2, dict):
    pct_hum = dict['pct_hum']
    mean_hum = dict['mean_hum']
    std_hum = dict['std_hum']
    mean_no = dict['mean_no']
    std_no = dict['std_no']
    euclidean = dict['euclidean_dist']

    # If there is nothing for mean_hum
    if not mean_hum:
        # and nothing for mean_no
        if not mean_no:
            # Return the euclidean distance, divided by 80% top speed (for time), plus two standard deviations
            return (euclidean / (0.22 * 0.8)) + (2 * 2.718)  # 2.718 is the average standard deviation across all edges
        # Otherwise return mean_no
        else:
            return mean_no + (2 * std_no)
    # if there is nothing in mean_no, return mean_hum
    elif not std_no:
        return mean_hum + (2 * std_hum)
    # Otherwise use the weighted sum of their means
    else:
        return pct_hum * (mean_hum + (2 * std_hum)) + (1 - pct_hum) * (mean_no + (2 * std_no))


def weight_function_3(n1, n2, dict):
    return dict['euclidean_dist']


def get_path(hosp_graph, n0, n1, weight_function):
    return dijkstra_path(hosp_graph, n0, n1, weight=weight_function)


if __name__ == '__main__':
    hosp_graph = HospGraph.unpickle_it(path_and_name)

    """
    Dijkstra can accept function calls as a weight. From documentation:
    If this is a function, the weight of an edge is the value returned by the function.
    The function must accept exactly three positional arguments:
    the two endpoints of an edge and the dictionary of edge attributes for that edge.
    The function must return a number.
    """
    print(dijkstra_path(hosp_graph, 'r00', 'r08', weight=weighted_means))
    print(dijkstra_path(hosp_graph, 'r00', 'r08', weight=mean_one_gaussian))
    print(dijkstra_path(hosp_graph, 'r00', 'r08', weight=weighted_95_percentile))
