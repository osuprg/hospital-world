#!/usr/bin/env python3
import STRUCT_hospital_graph_class as HospGraph
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from random import uniform

import POST_compare_choice_methods as Experiments
from POST_compare_choice_methods import PathData, AddPathInfo

class plot_hospital:
    def __init__(self, base_hosp_graph, experiments):
        addl_path_info = Experiments.AddPathInfo(base_hosp_graph, experiments)
        self.hosp_graph = addl_path_info.hosp_graph
        self.experiments = addl_path_info.experiments
        plt.axes()

    def add_to_plot(self):
        for node in self.hosp_graph.nodes():
            self.add_walls(node)
            self.add_nodes(node)

        for (n1, n2) in self.hosp_graph.edges():
            # self.add_dummy_weights(n1, n2)
            self.add_weights_from_data(n1, n2)

        plt.axis('scaled')
        plt.savefig('Path Comparison Plot')
        plt.clf()

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
        wall_rect = plt.Rectangle((wall_x_min, wall_y_min), wall_x_max - wall_x_min, wall_y_max - wall_y_min, edgecolor='black', fill=False)
        plt.gca().add_patch(wall_rect)

    def add_nodes(self, node):
        if 'ex' in node:
            return

        node_width = 0.1

        # This plots the locations of the nodes
        (xmin, xmax) = self.hosp_graph.nodes[node]['node_loc'][0].low, self.hosp_graph.nodes[node]['node_loc'][0].high
        (ymin, ymax) = self.hosp_graph.nodes[node]['node_loc'][1].low, self.hosp_graph.nodes[node]['node_loc'][1].high
        centroid = (xmin + ((xmax - xmin) / 2), ymin + ((ymax - ymin) / 2))
        self.hosp_graph.nodes[node]['centroid'] = centroid
        circle = plt.Circle(centroid, node_width, color='blue', fill=True)
        plt.gca().add_patch(circle)

    def add_weights_from_data(self, n1, n2):
        offset = 0
        colors = ['Purples', 'Greens', 'OrRd', 'Blues', 'Greys', 'Wistia']
        color_i = 0
        for method in self.experiments:
            edge_count = self.hosp_graph[n1][n2]['path_count_' + method.name]

            if edge_count < 1:
                continue

            x_interval = [self.hosp_graph.nodes[n1]['centroid'][0] + offset, self.hosp_graph.nodes[n2]['centroid'][0] + offset]
            y_interval = [self.hosp_graph.nodes[n1]['centroid'][1] + offset, self.hosp_graph.nodes[n2]['centroid'][1] + offset]
            c = cm.get_cmap(colors[color_i])

            plt.plot(x_interval, y_interval, c=c(edge_count / method.max_count), lw=2*(edge_count / method.max_count))
            offset += 0.15
            color_i += 1

    def add_dummy_weights(self, n1, n2):
        offset = [0.0, 0.1, 0.2]
        colors = ['Purples', 'Greens', 'OrRd']
        for i in range(3):

            rand_num = uniform(0, 1)
            if rand_num < 0.3:
                continue

            x_interval = [self.hosp_graph.nodes[n1]['centroid'][0] + offset[i], self.hosp_graph.nodes[n2]['centroid'][0] + offset[i]]
            y_interval = [self.hosp_graph.nodes[n1]['centroid'][1] + offset[i], self.hosp_graph.nodes[n2]['centroid'][1] + offset[i]]
            c = cm.get_cmap(colors[i])

            plt.plot(x_interval, y_interval, c=c(rand_num), lw=0.5)


if __name__ == "__main__":

    path_to_hosp_pickle = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/STRUCT_hospital_v1_param_pickle_2021-01-25'
    path_to_paths_pickle = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/paths_generated_2021-01-15_clean_data_run04'
    base_hosp_graph = HospGraph.unpickle_it(path_to_hosp_pickle)
    experiments = Experiments.unpickle_it(path_to_paths_pickle)

    hosp_plot = plot_hospital(base_hosp_graph, experiments)
    hosp_plot.add_to_plot()
