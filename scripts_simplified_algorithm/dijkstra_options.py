# Python things
import networkx as nx
from itertools import islice


class DijkstraMethod:
    """
    Dijkstra can accept function calls as a weight. From documentation:
    If this is a function, the weight of an edge is the value returned by the function.
    The function must accept exactly three positional arguments:
    the two endpoints of an edge and the dictionary of edge attributes for that edge.
    The function must return a number.

    In order to adjust this to also take in weights and different cost values, we wrap it
    in a class.
    """
    def __init__(self, hosp_graph):
        self.hosp_graph = hosp_graph
        self.w1 = 1
        self.w2 = 1
        self.val1 = 'mean_all'
        self.val2 = '95_all'

        self.path_vals = ['mean_all', 'std_all', 'mean_no', 'std_no', 'pct_hum', 'sq_dist', 'doors', 'nav_fail']
        self.weight_options = [1, 5, 10, 100, 1000]
        self.all_ratios = []
        self._ratio_pairs()

    def _ratio_pairs(self):
        # Get all ratio pairs
        for rat in reversed(self.weight_options):
            self.all_ratios.append((rat, 1))
        for rat in self.weight_options:
            if rat == 1:
                continue
            self.all_ratios.append((1, rat))

    def generic_function(self, n1, n2, dict):
        return self.w1 * dict[self.val1] + self.w2 * dict[self.val2]

    def janky_dijkstra(self, node1, node2):
        all_dij_paths = []
        # print('----------- {} to {} -----------'.format(node1, node2))

        # Loop through all pairs of path values
        for val1 in range(len(self.path_vals)):
            paths_generator = nx.shortest_simple_paths(self.hosp_graph, node1, node2, weight=self.path_vals[val1])
            shortest_simple = [i for i in islice(paths_generator, 20)]
            dummy_num = 0
            for j in shortest_simple:
                # fun_name = self.path_vals[val1] + str(dummy_num)
                all_dij_paths.append(j)
                dummy_num += 1

            for val2 in range(len(self.path_vals)):
                if val2 <= val1:
                    continue

                # Set the values in the class
                self.val1 = self.path_vals[val1]
                self.val2 = self.path_vals[val2]

                # Finally loop through all ratios
                for rat_num in range(len(self.all_ratios)):
                    # Get the current set of ratios and set the class ratios
                    (w1, w2) = self.all_ratios[rat_num]
                    self.w1 = w1
                    self.w2 = w2

                    # Function name is a combo of the value names and weights, for posterity
                    # fun_name = str(self.path_vals[val1]) + str(w1) + str(self.path_vals[val2]) + str(w2)
                    # Get the path based on the current ratio
                    dij_path = nx.dijkstra_path(self.hosp_graph, node1, node2, weight=self.generic_function)

                    all_dij_paths.append(dij_path)
        return all_dij_paths

    def less_janky_dijkstra(self, node1, node2):
        # Loop through all pairs of path values
        for val1 in range(len(self.path_vals)):
            for val2 in range(len(self.path_vals)):
                if val2 <= val1:
                    continue

                # Set the values in the class
                self.val1 = self.path_vals[val1]
                self.val2 = self.path_vals[val2]

                paths_generator = nx.shortest_simple_paths(self.hosp_graph, node1, node2, weight="mean_all")
                shortest_simple = [i for i in islice(paths_generator, 10)]
                for path in shortest_simple:
                    print(path)


