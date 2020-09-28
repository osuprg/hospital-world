import hospital_graph_class as hosp
import statistics
import matplotlib.pyplot as plt


def plot_it(n1, n2, data_to_plot):
    xmin = min(data_to_plot)
    xmax = max(data_to_plot)

    if xmax - xmin < 10:
        xmax = xmin + 10

    plt.xlim(xmin - 0.5, xmax)
    plt.hist(data_to_plot, bins=20)
    # plt.axis([0, 60, 0, 20])
    plt.title('Times between {} and {}. Data Points: {}'.format(n1, n2, len(data_to_plot)))
    plt.savefig('Times between {} and {}'.format(n1, n2))
    plt.clf()

if __name__ == "__main__":
    hospital = hosp.unpickle_it('hospital_interval_2020-06-01')
    # print(hospital.num_rooms)
    # print(hospital.G['r10_d00b']['r11_d00b']['weight'])
    for (n1, n2) in hospital.G.edges():
        rooms = ['r' + '%02d' % i for i in range(hospital.num_rooms)]
        if n1 in rooms or n2 in rooms or 'a' in n1 or 'a' in n2:
            continue

        # print('##########')
        # print('Edge between {} and {}'.format(n1, n2))
        data_to_play = hospital.G[n1][n2]['trav_data']
        if len(data_to_play) > 20:
            plot_it(n1, n2, data_to_play)

        try:
            pass
            # print('size of data', len(data_to_play))
            # print('spread', hospital.G[n1][n2]['weight'])
            # print('mean', statistics.mean(data_to_play))
            # print('median', statistics.median(data_to_play))
            # print('stdev', statistics.stdev(data_to_play))

        except statistics.StatisticsError:
            print("NO DATA")
            continue


