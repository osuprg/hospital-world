#!/usr/bin/env python3
import STRUCT_hospital_graph_class as HospGraph
import matplotlib.pyplot as plt


def plot_it(hosp_graph):
    plt.axes()
    for node in hosp_graph.nodes():
        print(node, hosp_graph.nodes[node]['node_loc'])
        (xmin, xmax) = hosp_graph.nodes[node]['node_loc'][0].low, hosp_graph.nodes[node]['node_loc'][0].high
        (ymin, ymax) = hosp_graph.nodes[node]['node_loc'][1].low, hosp_graph.nodes[node]['node_loc'][1].high

        start = (xmin, ymin)
        width = xmax - xmin
        height = ymax - ymin

        rect = plt.Rectangle(start, width, height, edgecolor='blue', fill=False)
        plt.text(start[0] + width/2, start[1] + height/2, node)
        plt.gca().add_patch(rect)

    plt.axis('scaled')
    plt.show()

if __name__ == "__main__":

    path_to_pickle = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/STRUCT_hospital_v1_param_pickle'
    hosp_graph = HospGraph.unpickle_it(path_to_pickle)

    plot_it(hosp_graph)
