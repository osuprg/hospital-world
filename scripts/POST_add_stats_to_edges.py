#!/usr/bin/env python3

#TODO All of this should be included in RUN_gather_edge_data.py in the future

import statistics
from numpy import sqrt, array
import STRUCT_hospital_graph_class as HospGraph

class AddStats:
    def __init__(self, path_and_name):
        self.top_speed = 0.22  # turtlebot's top speed is 0.22 m/s
        self.file_path = path_and_name
        self.hosp_graph = HospGraph.unpickle_it(self.file_path)

        self.tot_start = 0
        self.tot_end = 0

    def main(self):
        for (n1, n2) in self.hosp_graph.edges():
            print("--------------------")
            print(n1, n2)
            array_no = self.hosp_graph[n1][n2]['trav_data_no']
            array_hum = self.hosp_graph[n1][n2]['trav_data_hum']
            if len(array_no + array_hum) < 2:
                self.hosp_graph.remove_edge(n1, n2)
                print('edge removed between {} and {}'.format(n1, n2))
                continue
            self.euclidean_dist(n1, n2)
            self.min_distance(n1, n2)
            self.clean_data(n1, n2)
            self.add_stats(n1, n2)

        self.nav_failure_count()

        print("total removed: {} of {}".format(self.tot_start - self.tot_end, self.tot_start))
        HospGraph.pickle_it(self.hosp_graph, self.file_path + '_plus_stats_clean')

    def euclidean_dist(self, n1, n2):
        n1_loc = self.hosp_graph.nodes[n1]['node_loc']
        n2_loc = self.hosp_graph.nodes[n2]['node_loc']
        # print(n1_loc[0].low)
        # Gets the difference between the centers of each node
        x_diff = ((n1_loc[0].low + n1_loc[0].high) / 2) - ((n2_loc[0].low + n2_loc[0].high) / 2)
        y_diff = ((n1_loc[1].low + n1_loc[1].high) / 2) - ((n2_loc[1].low + n2_loc[1].high) / 2)
        self.hosp_graph[n1][n2]['euclidean_dist'] = sqrt(x_diff ** 2 + y_diff ** 2)
        self.hosp_graph[n1][n2]['sq_dist'] = abs(x_diff) + abs(y_diff)

    def min_distance(self, n1, n2):
        [n1_x, n1_y] = self.hosp_graph.nodes[n1]['node_loc']
        [n2_x, n2_y] = self.hosp_graph.nodes[n2]['node_loc']

        if n1_x.high < n2_x.low:    # Node 1 is left of node 2
            x_dim = n2_x.low - n1_x.high
        elif n1_x.low > n2_x.high:  # Node 1 is right of node 2
            x_dim = n1_x.low - n2_x.high
        else:                       # Node 1 and node 2 overlap - no difference in the x direction at the closest points
            x_dim = 0

        if n1_y.high < n2_y.low:    # Node 1 is below node 2
            y_dim = n2_y.low - n1_y.high
        elif n1_y.low > n2_y.high:  # Node 1 is above node 2
            y_dim = n1_y.low - n2_y.high
        else:                       # Node 1 and node 2 overlap - no difference in the y direction at the closest points
            y_dim = 0

        self.hosp_graph[n1][n2]['min_dist'] = sqrt(x_dim ** 2 + y_dim ** 2)

    def add_stats(self, n1, n2):
        array_no = self.hosp_graph[n1][n2]['trav_data_no']
        array_hum = self.hosp_graph[n1][n2]['trav_data_hum']

        self.hosp_graph[n1][n2]['nav_fail'] = 0
        self.hosp_graph[n1][n2]['doors'] = 0

        if 'd' in n1:
            self.hosp_graph[n1][n2]['doors'] += 0.5
        if 'd' in n2:
            self.hosp_graph[n1][n2]['doors'] += 0.5

        # Need separate blocks because one or the other may not have data
        try:
            self.hosp_graph[n1][n2]['mean_all'] = statistics.mean(array_no + array_hum)
            self.hosp_graph[n1][n2]['std_all'] = statistics.stdev(array_no + array_hum)
            self.hosp_graph[n1][n2]['95_all'] = self.hosp_graph[n1][n2]['mean_all'] + 2 * self.hosp_graph[n1][n2]['std_all']
        except statistics.StatisticsError:
            self.hosp_graph[n1][n2]['mean_all'] = None
            self.hosp_graph[n1][n2]['std_all'] = None
            self.hosp_graph[n1][n2]['95_all'] = None

        try:
            self.hosp_graph[n1][n2]['mean_no'] = statistics.mean(array_no)
            self.hosp_graph[n1][n2]['std_no'] = statistics.stdev(array_no)
            self.hosp_graph[n1][n2]['95_no'] = self.hosp_graph[n1][n2]['mean_no'] + 2 * self.hosp_graph[n1][n2]['std_no']
        except statistics.StatisticsError:
            self.hosp_graph[n1][n2]['mean_no'] = None
            self.hosp_graph[n1][n2]['std_no'] = None
            self.hosp_graph[n1][n2]['95_no'] = None

        try:
            self.hosp_graph[n1][n2]['mean_hum'] = statistics.mean(array_hum)
            self.hosp_graph[n1][n2]['std_hum'] = statistics.stdev(array_hum)
            self.hosp_graph[n1][n2]['95_hum'] = self.hosp_graph[n1][n2]['mean_hum'] + 2 * self.hosp_graph[n1][n2]['std_hum']
        except statistics.StatisticsError:
            self.hosp_graph[n1][n2]['mean_hum'] = None
            self.hosp_graph[n1][n2]['std_hum'] = None
            self.hosp_graph[n1][n2]['95_hum'] = None

        try:
            self.hosp_graph[n1][n2]['pct_hum'] = len(array_hum) / (len(array_hum) + len(array_no))
        except ZeroDivisionError:
            self.hosp_graph[n1][n2]['pct_hum'] = 0

        self.hosp_graph[n1][n2]['hum_dist'] = self.hosp_graph[n1][n2]['pct_hum'] * self.hosp_graph[n1][n2]['sq_dist']

        print('mean_all', self.hosp_graph[n1][n2]['mean_all'])

    def nav_failure_count(self):
        fake_it_list = [('h01', 'h02'), ('r17_d00', 'r16_d00'), ('h01', 'r02_d00'), ('r01_d01', 'r02_d00')]

        for (n1, n2) in fake_it_list:
            self.hosp_graph[n1][n2]['nav_fail'] += 1

    def clean_data(self, n1, n2):

        # Theoretically the minimum amount of time it could take to travel between two nodes
        min_val = self.hosp_graph[n1][n2]['min_dist'] / self.top_speed
        # min_val = 0
        max_val = 300

        start_no = len(self.hosp_graph[n1][n2]['trav_data_no'])
        self.tot_start += start_no
        new_arr = [j for j in self.hosp_graph[n1][n2]['trav_data_no'] if min_val <= j <= max_val]
        self.hosp_graph[n1][n2]['trav_data_no'] = new_arr
        end_no = len(self.hosp_graph[n1][n2]['trav_data_no'])
        self.tot_end += end_no

        if start_no != end_no:
            print("NO, start: {} | end: {}".format(start_no, end_no))
            try:
                print('min_no', min(new_arr))
            except ValueError:
                pass

        start_hum = len(self.hosp_graph[n1][n2]['trav_data_hum'])
        self.tot_start += start_hum
        new_arr_hum = [i for i in self.hosp_graph[n1][n2]['trav_data_hum'] if min_val <= i <= max_val]
        dummy_arr_num_max = [j for j in self.hosp_graph[n1][n2]['trav_data_hum'] if j > max_val]
        if len(dummy_arr_num_max) > 0:
            print(dummy_arr_num_max)
        self.hosp_graph[n1][n2]['trav_data_hum'] = new_arr_hum
        self.hosp_graph[n1][n2]['count_nav_fail'] = len(dummy_arr_num_max)
        end_hum = len(self.hosp_graph[n1][n2]['trav_data_hum'])
        self.tot_end += end_hum

        if start_hum != end_hum:
            print("HUM, start: {} | end: {}".format(start_hum, end_hum))
            # print(min(new_arr_hum))
            try:
                print('min_hum', min(new_arr_hum))
            except ValueError:
                pass


if __name__ == "__main__":

    # Yes this is terribly dumb, but you have to write out the full filepath
    # Otherwise it will save to the directory from which you are running the script
    path_and_name = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/hospital_trials_2021-02-11_hum_50_70'

    add_stats_cl = AddStats(path_and_name)
    add_stats_cl.main()
