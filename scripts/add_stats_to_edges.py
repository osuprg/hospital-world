#!/usr/bin/env python3

#TODO All of this should be included in RUN_gather_edge_data.py in the future

import statistics
import STRUCT_hospital_graph_class as HospGraph



if __name__ == "__main__":

    # Yes this is terribly dumb, but you have to write out the full filepath
    # Otherwise it will save to the directory from which you are running the script
    path_and_name = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/hospital_trials_2020-10-23'
    hosp_graph = HospGraph.unpickle_it(path_and_name)

    for (n1, n2) in hosp_graph.edges():
        array_no = hosp_graph[n1][n2]['trav_data_no']
        array_hum = hosp_graph[n1][n2]['trav_data_hum']

        # Need separate blocks because one or the other may not have data
        try:
            hosp_graph[n1][n2]['mean_no'] = statistics.mean(array_no)
            hosp_graph[n1][n2]['std_no'] = statistics.stdev(array_no)
        except statistics.StatisticsError:
            hosp_graph[n1][n2]['mean_no'] = None
            hosp_graph[n1][n2]['std_no'] = None

        try:
            hosp_graph[n1][n2]['mean_hum'] = statistics.mean(array_hum)
            hosp_graph[n1][n2]['std_hum'] = statistics.stdev(array_hum)
        except statistics.StatisticsError:
            hosp_graph[n1][n2]['mean_hum'] = None
            hosp_graph[n1][n2]['std_hum'] = None

        try:
            hosp_graph[n1][n2]['pct_hum'] = len(array_hum) / (len(array_hum) + len(array_no))
        except ZeroDivisionError:
            hosp_graph[n1][n2]['pct_hum'] = None

        #
        # print(n1, n2, hosp_graph[n1][n2])

    HospGraph.pickle_it(hosp_graph, path_and_name+'_plus_stats')


