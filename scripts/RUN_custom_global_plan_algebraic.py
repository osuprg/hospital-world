#!/usr/bin/env python3

import STRUCT_hospital_graph_class as HospGraph
from networkx import dijkstra_path

#TODO: The statistics should be calculated in the file that collects / saves data
import statistics

# Yes this is terribly dumb, but you have to write out the full filepath
# Otherwise it will save to the directory from which you are running the script
path_and_name = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/hospital_trials_2020-10-23_plus_stats'



def weight_function_1(n1, n2, dict):
    pct_hum = dict['pct_hum']
    mean_hum = dict['mean_hum']
    mean_no = dict['mean_no']

    # If there is nothing for mean_hum
    if not mean_hum:
        # and nothing for mean_no
        if not mean_no:
            # Return a huge number
            return 10000
        # Otherwise return mean_no
        else:
            return mean_no
    # if there is nothing in mean_no, return mean_hum
    elif not mean_no:
        return mean_hum
    # Otherwise use the weighted sum of their means
    else:
        return pct_hum * mean_hum + (1 - pct_hum) * mean_no

def weight_function_2(n1, n2, dict):
    pct_hum = dict['pct_hum']
    std_hum = dict['std_hum']
    std_no = dict['std_no']

    # If there is nothing for mean_hum
    if not std_hum:
        # and nothing for mean_no
        if not std_no:
            # Return a huge number
            return 10000
        # Otherwise return mean_no
        else:
            return std_no
    # if there is nothing in mean_no, return mean_hum
    elif not std_no:
        return std_hum
    # Otherwise use the weighted sum of their means
    else:
        return pct_hum * std_hum + (1 - pct_hum) * std_no

def weight_function_3(n1, n2, dict):
    return dict['pct_hum']

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
    print(dijkstra_path(hosp_graph, 'r00', 'r08', weight=weight_function_1))
    print(get_path(hosp_graph, 'r00', 'r08', weight_function_1))

    print(dijkstra_path(hosp_graph, 'r00', 'r08', weight=weight_function_2))
    print(get_path(hosp_graph, 'r00', 'r08', weight_function_2))

    print(dijkstra_path(hosp_graph, 'r00', 'r08', weight=weight_function_3))
    print(get_path(hosp_graph, 'r00', 'r08', weight_function_3))
