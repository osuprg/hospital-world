#!/usr/bin/env python3
import STRUCT_hospital_graph_class as HospGraph
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from random import uniform

class plot_hospital:
    def __init__(self, hosp_graph):
        self.hosp_graph = hosp_graph
        plt.axes()

    def add_to_plot(self):
        for node in self.hosp_graph.nodes():
            self.add_walls(node)
            self.add_nodes(node)

        for (n1, n2) in self.hosp_graph.edges():
            offset = 0
            self.add_weights_01(n1, n2, offset)

        plt.axis('scaled')
        plt.show()

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

    def add_weights_01(self, n1, n2, offset):
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

    path_to_pickle = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/STRUCT_hospital_v1_param_pickle_2021-01-25'
    hosp_graph = HospGraph.unpickle_it(path_to_pickle)

    hosp_plot = plot_hospital(hosp_graph)
    hosp_plot.add_to_plot()
