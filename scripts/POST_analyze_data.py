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


def plot_mult(n1, n2, data_no, data_hum, mean_no, mean_hum):
    if len(data_no) < 5 and len(data_hum) < 5:
        return

    x_min = 0
    x_max = 1
    try:
        plt.hist(data_no, bins=50, alpha=0.5, label='no')
        plt.axvline(mean_no, color='b', linestyle='dashed', label='mean_no')
        x_min = min(data_no)
        x_max = max(data_no)
        plt.axvline(x_min, color='lightblue', linestyle='dotted', label='min_no')
        plt.axvline(x_max, color='lightblue', linestyle='dotted', label='max_no')
    except TypeError:
        pass
    try:
        plt.hist(data_hum, bins=50, alpha=0.5, label='hum')
        plt.axvline(mean_hum, color='r', linestyle='dashed', label='mean_hum')
        hum_xmin = min(data_hum)
        hum_xmax = max(data_hum)
        if hum_xmin < x_min:
            x_min = min(data_hum)
        if hum_xmax > x_max:
            x_max = max(data_hum)
        plt.axvline(hum_xmin, color='lightsalmon', linestyle='dotted', label='min_hum')
        plt.axvline(hum_xmax, color='lightsalmon', linestyle='dotted', label='max_hum')
    except TypeError:
        pass

    x_min -= 0.5
    x_max += 0.5

    if x_max - x_min < 10:
        mid = (x_max + x_min) / 2
        x_min = mid - 5
        x_max = mid + 5
    if x_min < 0:
        x_min = 0

    plt.xlim(x_min, x_max)
    # plt.axis([0, 60, 0, 20])
    plt.title('Times between {} and {}. Data Points - no: {}, hum: {}'.format(n1, n2, len(data_no), len(data_hum)))
    plt.legend(loc='upper right')
    plt.savefig('Times between {} and {}'.format(n1, n2))
    plt.clf()


if __name__ == "__main__":
    path_to_pickle = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/hospital_trials_2021-02-11_hum_50_70_plus_stats_clean'
    hosp_graph = HospGraph.unpickle_it(path_to_pickle)

    mean_no_all = 0
    tot_no_all = 0
    mean_hum_all = 0
    tot_hum_all = 0

    for (n1, n2) in hosp_graph.edges():
        rooms = ['r' + '%02d' % i for i in range(hosp_graph.graph['num_rooms'])]
        # if n1 in rooms or n2 in rooms or 'a' in n1 or 'a' in n2:
        #     continue

        cond_no = 'no'
        cond_hum = 'hum'
        data_no = hosp_graph[n1][n2]['trav_data_' + cond_no]
        data_hum = hosp_graph[n1][n2]['trav_data_' + cond_hum]
        mean_no = hosp_graph[n1][n2]['mean_' + cond_no]
        mean_hum = hosp_graph[n1][n2]['mean_' + cond_hum]
        print('---------------------')
        print(n1, n2)

        try:
            mean_no_all += sum(data_no)
            tot_no_all += len(data_no)
        except TypeError:
            pass
        try:
            mean_hum_all += sum(data_hum)
            tot_hum_all += len(data_hum)
        except TypeError:
            pass

        try:
            print("no: [{}, {}]".format(min(data_no), max(data_no)))
        except ValueError:
            print('no data does not have a min or max')

        try:
            print("hum: [{}, {}]".format(min(data_hum), max(data_hum)))
        except ValueError:
            print('human data does not have a min or max')

        plot_mult(n1, n2, data_no, data_hum, mean_no, mean_hum)


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


