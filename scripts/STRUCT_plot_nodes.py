#!/usr/bin/env python3
import STRUCT_hospital_graph_class as HospGraph
import matplotlib.pyplot as plt
# import matplotlib.cm as cm
# from random import uniform
#
# import POST_compare_choice_methods as Experiments
# from POST_compare_choice_methods import PathData, AddPathInfo

class plot_hospital:
    def __init__(self, hosp_graph):
        self.hosp_graph = hosp_graph
        plt.axes()

        self.label_num = 0

    def add_to_plot(self):   #, n1, n2, all_paths):
        for node in self.hosp_graph.nodes():
            self.add_walls(node)
            self.add_nodes(node)
            self.label_num += 1

        self.label_num = 0

        self.add_hum_cond()

        for (n1, n2) in self.hosp_graph.edges():
            # self.add_dummy_weights(n1, n2)
            # self.add_weights_from_data(n1, n2)
            self.add_edges(n1, n2)
            self.label_num += 1

        # self.add_edge_weights(all_paths)
        # self.add_nav_fail()
        plt.axis('scaled')
        plt.legend(loc=3, framealpha=1, ncol=5)
        plt.title('Environment with node and edge embeddings')
        plt.tight_layout()
        plt.savefig('Environment with node and edge embeddings.svg', dpi=500)
        plt.clf()

    def add_edges(self, n1, n2):
        (x1, y1) = self.hosp_graph.nodes[n1]['centroid']
        (x2, y2) = self.hosp_graph.nodes[n2]['centroid']

        x_interval = [x1, x2]
        y_interval = [y1, y2]
        if self.label_num == 0:
            plt.plot(x_interval, y_interval, c='red', label='Edges', lw=1)
        else:
            plt.plot(x_interval, y_interval, c='red', lw=1)

    def add_walls(self, node):
        # This plots the basic outlines of the rooms and walls
        try:
            [[rm_x_min, rm_x_max], [rm_y_min, rm_y_max]] = self.hosp_graph.nodes[node]['rm_loc']
            start = (rm_x_min, rm_y_min)
            width = rm_x_max - rm_x_min
            height = rm_y_max - rm_y_min
            rect = plt.Rectangle(start, width, height, edgecolor='black', fill=False)
            plt.gca().add_patch(rect)
        except KeyError:
            pass

        # This is the wall near halls 0 and 3
        wall_x_min = 5
        wall_x_max = 5
        wall_y_min = 5
        wall_y_max = 14
        if self.label_num == 0:
            wall_rect = plt.Rectangle((wall_x_min, wall_y_min), wall_x_max - wall_x_min, wall_y_max - wall_y_min,
                                      edgecolor='black', fill=False, label='Walls')
        else:
            wall_rect = plt.Rectangle((wall_x_min, wall_y_min), wall_x_max - wall_x_min, wall_y_max - wall_y_min, edgecolor='black', fill=False)
        plt.gca().add_patch(wall_rect)

    def add_nodes(self, node):
        if 'ex' in node:
            return
        add_to_y = ['h00', 'h01']
        sub_from_y = ['h02', 'h03']
        node_width = 0.2
        if node in add_to_y:
            # This plots the locations of the nodes
            (xmin, xmax) = self.hosp_graph.nodes[node]['node_loc'][0].low, self.hosp_graph.nodes[node]['node_loc'][0].high
            (ymin, ymax) = self.hosp_graph.nodes[node]['node_loc'][1].low + 0.8, self.hosp_graph.nodes[node]['node_loc'][1].high + 0.8
            centroid = (xmin + ((xmax - xmin) / 2), ymin + ((ymax - ymin) / 2))
        elif node in sub_from_y:
            # This plots the locations of the nodes
            (xmin, xmax) = self.hosp_graph.nodes[node]['node_loc'][0].low, self.hosp_graph.nodes[node]['node_loc'][0].high
            (ymin, ymax) = self.hosp_graph.nodes[node]['node_loc'][1].low - 1.2, self.hosp_graph.nodes[node]['node_loc'][1].high - 1.2
            centroid = (xmin + ((xmax - xmin) / 2), ymin + ((ymax - ymin) / 2))
        else:
            # This plots the locations of the nodes
            (xmin, xmax) = self.hosp_graph.nodes[node]['node_loc'][0].low, self.hosp_graph.nodes[node]['node_loc'][0].high
            (ymin, ymax) = self.hosp_graph.nodes[node]['node_loc'][1].low, self.hosp_graph.nodes[node]['node_loc'][1].high
            centroid = (xmin + ((xmax - xmin) / 2), ymin + ((ymax - ymin) / 2))
        self.hosp_graph.nodes[node]['centroid'] = centroid

        if self.label_num == 0:
            circle = plt.Circle(centroid, node_width, color='blue', fill=True, label="Nodes", zorder=10)
        else:
            circle = plt.Circle(centroid, node_width, color='blue', fill=True, zorder=10)
        plt.gca().add_patch(circle)

    def add_nav_fail(self):
        nav_fail = [[3.5, 5], [14.5, 11], [20, 14.5]]
        node_width = 0.1
        for [x, y] in nav_fail:
            plt.scatter(x, y, color='orange', marker=6)

    def add_hum_cond(self):
        cond_01 = [[[5, 26], [14, 16]], [[14, 17], [5, 14]]]
        cond_02 = [[[5, 26], [3, 5]], [[5, 10], [5, 14]]]
        i = 0
        for [x, y] in cond_01:
            start = (x[0], y[0])
            width = x[1] - x[0]
            height = y[1] - y[0]
            if i == 0:
                hum_cond = plt.Rectangle(start, width, height, color='blue', alpha=0.2, linewidth=0, label='Cond 1')
            else:
                hum_cond = plt.Rectangle(start, width, height, color='blue', alpha=0.2, linewidth=0)
            plt.gca().add_patch(hum_cond)
            i += 1

        i = 0
        for [x, y] in cond_02:
            start = (x[0], y[0])
            width = x[1] - x[0]
            height = y[1] - y[0]
            if i == 0:
                hum_cond = plt.Rectangle(start, width, height, color='red', alpha=0.2, linewidth=0, label='Cond 2')
            else:
                hum_cond = plt.Rectangle(start, width, height, color='red', alpha=0.2, linewidth=0)
            plt.gca().add_patch(hum_cond)
            i += 1

    def add_weights_from_data(self, n1, n2):
        # Old method, keeping for posterity.
        pass
        # addl_path_info = Experiments.AddPathInfo(base_hosp_graph, experiments)
        # self.hosp_graph = addl_path_info.hosp_graph
        # self.experiments = addl_path_info.experiments
        # offset = 0
        # colors = ['Purples', 'Greens', 'OrRd', 'Blues', 'Greys', 'Wistia']
        # color_i = 0
        # for method in self.experiments:
        #     edge_count = self.hosp_graph[n1][n2]['path_count_' + method.name]
        #
        #     if edge_count < 1:
        #         continue
        #
        #     x_interval = [self.hosp_graph.nodes[n1]['centroid'][0] + offset, self.hosp_graph.nodes[n2]['centroid'][0] + offset]
        #     y_interval = [self.hosp_graph.nodes[n1]['centroid'][1] + offset, self.hosp_graph.nodes[n2]['centroid'][1] + offset]
        #     c = cm.get_cmap(colors[color_i])
        #
        #     plt.plot(x_interval, y_interval, c=c(edge_count / method.max_count), lw=2*(edge_count / method.max_count))
        #     offset += 0.15
        #     color_i += 1

    def add_edge_weights(self, paths):
        colors = ['royalblue', 'maroon', 'darkgreen', 'darkgoldenrod']
        offset = [x * 0.1 for x in range(len(paths))]
        y = 0

        for j in range(len(paths)):
            offset = 0.15 * j

            [name, path] = paths[j]

            for i in range(len(path) - 1):
                n1 = path[i]
                n2 = path[i + 1]

                x_interval = [self.hosp_graph.nodes[n1]['centroid'][0] + offset, self.hosp_graph.nodes[n2]['centroid'][0] + offset]
                y_interval = [self.hosp_graph.nodes[n1]['centroid'][1] + offset, self.hosp_graph.nodes[n2]['centroid'][1] + offset]
                if i == 0:
                    plt.plot(x_interval, y_interval, label=name, c=colors[j], lw=1.5)
                else:
                    plt.plot(x_interval, y_interval, c=colors[j], lw=1.5)

if __name__ == "__main__":

    path_to_hosp_pickle = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/STRUCT_hospital_v1_param_pickle_2021-01-25'
    # path_to_paths_pickle = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/paths_generated_2021-01-15_clean_data_run04'
    unpickled_hosp_graph = HospGraph.unpickle_it(path_to_hosp_pickle)

    hosp_plot = plot_hospital(unpickled_hosp_graph)

    hosp_plot.add_to_plot()
    # paths_00_10 = [['Not slow', ['r00', 'r00_d00', 'r01_d00', 'r01_d01', 'r02_d00', 'r03_d00', 'r05_d00', 'r06_d00', 'r07_d00', 'r08_d00', 'r09_d00', 'r10_d00', 'r10']],
    #                ['Delicate', ['r00', 'r00_d00', 'r01_d00', 'r01_d01', 'r02_d00', 'r03_d00', 'r05_d00', 'r06_d00', 'r07_d00', 'r08_d00', 'r09_d00', 'r10_d00', 'r10']],
    #                ['After hours', ['r00', 'r00_d00', 'r01_d00', 'r01_d01', 'h01', 'h02', 'r11_d00', 'r10_d00', 'r10']]]
    #
    # paths_00_11 = [['Not slow', ['r00', 'r00_d00', 'r01_d00', 'r01_d01', 'h01', 'h02', 'r11_d00', 'r11']],
    #                ['Delicate', ['r00', 'r00_d00', 'r01_d00', 'r01_d01', 'h01', 'h02', 'r11_d00', 'r11']],
    #                ['After hours', ['r00', 'r00_d00', 'r01_d00', 'r01_d01', 'h01', 'h02', 'r11_d00', 'r11']]]
    #
    # paths_00_12 = [['Not slow', ['r00', 'r00_d00', 'r01_d00', 'h00', 'h03', 'r13_d00', 'r12_d00', 'r12']],
    #                ['Delicate', ['r00', 'r00_d00', 'r20_d00', 'r19_d00', 'r18_d00', 'r17_d00', 'r16_d00', 'r15_d00', 'r14_d00', 'r13_d00', 'r12_d00', 'r12']],
    #                ['After hours', ['r00', 'r00_d00', 'r01_d00', 'h00', 'h03', 'r13_d00', 'r12_d00', 'r12']]]
    #
    # paths_02_13 = [['Not slow', ['r02', 'r02_d00', 'h01', 'h02', 'r11_d00', 'r12_d00', 'r13_d00', 'r13']],
    #                ['Delicate', ['r02', 'r02_d00', 'h01', 'h02', 'r11_d00', 'r12_d00', 'r13_d00', 'r13']],
    #                ['After hours', ['r02', 'r02_d00', 'h01', 'h02', 'r11_d00', 'r12_d00', 'r13_d00', 'r13']]]
    #
    # paths_02_14 = [['Not slow', ['r02', 'r02_d00', 'r01_d01', 'h00', 'h03', 'r14_d00', 'r14']],
    #                ['Delicate', ['r02', 'r02_d00', 'r01_d01', 'r01_d00', 'r00_d00', 'r20_d00', 'r19_d00', 'r18_d00', 'r17_d00', 'r16_d00', 'r15_d00', 'r14_d00', 'r14']],
    #                ['After hours', ['r02', 'r02_d00', 'r01_d01', 'h00', 'h03', 'r14_d00', 'r14']]]
    #
    # paths_02_16 = [['Not slow', ['r02', 'r02_d00', 'r01_d01', 'r01_d00', 'r00_d00', 'r20_d00', 'r19_d00', 'r18_d00', 'r17_d00', 'r16_d00', 'r16']],
    #                ['Delicate', ['r02', 'r02_d00', 'r01_d01', 'r01_d00', 'r00_d00', 'r20_d00', 'r19_d00', 'r18_d00', 'r17_d00', 'r16_d00', 'r16']],
    #                ['After hours', ['r02', 'r02_d00', 'r01_d01', 'h00', 'h03', 'r14_d00', 'r15_d00', 'r16_d00', 'r16']]]
    #
    # #
    # # paths_03 = [['Not slow', ['r03', 'r03_d00', 'r02_d00', 'r01_d01', 'h00', 'h03', 'r14_d00', 'r14']],
    # #             ['Delicate', ['r03', 'r03_d00', 'r02_d00', 'r01_d01', 'r01_d00', 'r00_d00', 'r20_d00', 'r19_d00', 'r18_d00', 'r17_d00', 'r16_d00', 'r15_d00', 'r14_d00', 'r14']],
    # #             ['After hours', ['r03', 'r03_d00', 'r02_d00', 'r01_d01', 'h00', 'h03', 'r14_d00', 'r14']]]
    # #
    # # paths_04 = [['Not slow', ['r05', 'r05_d00', 'r03_d00', 'r02_d00', 'h01', 'h02', 'r11_d00', 'r11']],
    # #             ['Delicate', ['r05', 'r05_d00', 'r06_d00', 'r07_d00', 'r08_d00', 'r09_d00', 'r10_d00', 'r11_d00', 'r11']],
    # #             ['After hours', ['r05', 'r05_d00', 'r03_d00', 'r02_d00', 'h01', 'h02', 'r11_d00', 'r11']]]
    # #
    # # paths_05 = [['Not slow', ['r06', 'r06_d00', 'r07_d00', 'r08_d00', 'r09_d00', 'r10_d00', 'r11_d00', 'r12_d00', 'r13_d00', 'r14_d00', 'r15_d00', 'r15']],
    # #             ['Delicate', ['r06', 'r06_d00', 'r05_d00', 'r03_d00', 'r02_d00', 'r01_d01', 'r01_d00', 'r00_d00', 'r20_d00', 'r19_d00', 'r18_d00', 'r17_d00', 'r16_d00', 'r15_d00', 'r15']],
    # #             ['After hours', ['r06', 'r06_d00', 'r07_d00', 'r08_d00', 'r09_d00', 'r10_d00', 'r11_d00', 'r12_d00', 'r13_d00', 'r14_d00', 'r15_d00', 'r15']]]
    #
    #
    # paths_to_plot = [['r00', 'r10', paths_00_10],
    #                  ['r00', 'r11', paths_00_11],
    #                  ['r00', 'r12', paths_00_12],
    #                  ['r02', 'r13', paths_02_13],
    #                  ['r02', 'r14', paths_02_14],
    #                  ['r02', 'r16', paths_02_16]]
    #
    #
    # for [n1, n2, paths] in paths_to_plot:
    #     hosp_plot.add_to_plot(n1, n2, paths)
