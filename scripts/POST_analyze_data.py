#!/usr/bin/env python2


import STRUCT_hospital_graph_class as HospGraph
import statistics
import matplotlib.pyplot as plt


def plot_it(n1, n2, data_to_plot, cond):
    # xmin = min(data_to_plot)
    # xmax = max(data_to_plot)
    #
    # if xmax - xmin < 10:
    #     xmax = xmin + 10
    #
    # plt.xlim(xmin - 0.5, xmax)
    plt.hist(data_to_plot, bins=50)
    # plt.axis([0, 60, 0, 20])
    plt.title('Times between {} and {}. Data Points: {}. Condition: {}'.format(n1, n2, len(data_to_plot), cond))
    plt.savefig('Times between {} and {} cond {}'.format(n1, n2, cond))
    plt.clf()

def plot_mult(n1, n2, data_no, data_hum):
    plt.hist(data_no, bins=50, alpha=0.5, label='no')
    plt.hist(data_hum, bins=50, alpha=0.5, label='hum')

    # plt.axis([0, 60, 0, 20])
    plt.title('Times between {} and {}. Data Points - no: {}, hum: {}'.format(n1, n2, len(data_no), len(data_hum)))
    plt.legend(loc='upper right')
    plt.savefig('Times between {} and {}'.format(n1, n2))
    plt.clf()

if __name__ == "__main__":
    path_to_pickle = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/hospital_trials_2020-12-19_plus_stats'
    hosp_graph = HospGraph.unpickle_it(path_to_pickle)

    for (n1, n2) in hosp_graph.edges():
        rooms = ['r' + '%02d' % i for i in range(hosp_graph.graph['num_rooms'])]
        if n1 in rooms or n2 in rooms or 'a' in n1 or 'a' in n2:
            continue

        cond_no = 'no'
        cond_hum = 'hum'
        data_no = hosp_graph[n1][n2]['trav_data_' + cond_no]
        data_hum = hosp_graph[n1][n2]['trav_data_' + cond_hum]
        plot_mult(n1, n2, data_no, data_hum)


        # try:
        #     pass
            # print('size of data', len(data_to_play))
            # print('spread', hospital.G[n1][n2]['weight'])
            # print('mean', statistics.mean(data_to_play))
            # print('median', statistics.median(data_to_play))
            # print('stdev', statistics.stdev(data_to_play))

        # except statistics.StatisticsError:
        #     print("NO DATA")
        #     continue


