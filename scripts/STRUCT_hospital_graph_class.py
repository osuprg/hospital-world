#!/usr/bin/env python2

import networkx as nx
from networkx.drawing.nx_agraph import graphviz_layout
import matplotlib.pyplot as plt
import numpy as np
import pickle
from STRUCT_interval_cust import Interval as cust_interval
from os import getcwd
import pickle

"""
Rooms are in the format 'r##'
Doors are 'r##_d##a' room, room #, door, door #, node A or B
Halls are 'h##'
"""

class HospitalGraph:
    def __init__(self, num_rooms=None, num_halls=None, extra_doors=None, hall_door_links=None, hall_room_extra=None,
                 connected_halls=None, connected_rooms=False, filename=None, init_pose=(0, 0)):
        """

        Args:
            num_rooms: Integer for number of rooms
            num_halls: Integer for number of halls
            extra_doors: List of rooms with extra doors - list a room once for each extra door, e.g. ['r2', 'r2', 'r3']
            hall_door_links: List of which primary doors (d00) connect to which hall in tuple pairs, e.g. [('r02_d00b', 'h2')]
            hall_room_extra: List of which additional doors (d01+) connect to which halls in tuple pairs
            connected_halls: List of which halls connect together, e.g. [('h01', 'h02')]
        """
        self.G = nx.Graph(initial_pose=init_pose)
        self.num_rooms = num_rooms              # Integer
        self.G.graph['num_rooms'] = num_rooms
        self.num_halls = num_halls              # Integer
        self.extra_doors = extra_doors          # List of rooms with extra doors - list a room once for each extra door
        self.rooms = None                       # Contains string representations of all room nodes
        self.halls = None                       # Contains string representations of all hall nodes
        self.doors = []                         # Contains string representations of all door nodes
        self.hall_room_links = hall_door_links  # List of which halls and doors are connected
        self.hall_room_extra = hall_room_extra  # List of extra doors (i.e. not door 0) are connected to what halls
        self.connected_halls = connected_halls  # List of which halls are connected together
        self.connected_rooms = connected_rooms  # Bool to determine if all sequential rooms are connected

        self.hall_width = 2.0
        self.node_width = 0.5
        self.hall_node_width = 1.0

        self.filename = filename
        self.data = None

        self.read_file()
        self.build_graph()

    def read_file(self):
        with open(self.filename, 'r') as f:
            self.data = []
            for line in f:
                self.data.append(line)

    def build_graph(self):
        # self._draw_nodes_old()                       # Draw all the nodes

        for line in self.data:
            parts = line.split()
            node_info = []

            # Ignore comment lines
            if parts[0] is '#':
                continue

            # Convert numbers to floats
            for p in parts:
                try:
                    node_info.append(float(p))
                except ValueError:
                    node_info.append(p)

            if node_info[0][0] == 'r':
                self._add_room_loc(node_info)
                self._add_door_loc(node_info)

            elif node_info[0][0] == 'h':
                self._add_hall_loc(node_info)

            elif node_info[0][0] == 'e':
                self._add_excl_loc(node_info)

            else:
                print('not sure what to do with that node type:', node_info[0])

        self._draw_edges()                       # Connect the appropriate nodes
        self._add_edge_weights()                      # Add weights to the edges

    def _draw_edges(self):

        for door in self.doors:

            # Add edges between interior ('a') door node and the associated room
            for r in range(self.num_rooms):
                room = 'r' + '%02d' % r
                # weight = r
                if room in door and 'a' in door:
                    self.G.add_edge(room, door)

            # Add edges between interior ('a') and exterior ('b') doors
            for door_b in self.doors:
                if door_b[:-1] in door and door_b is not door:
                    if not self.G.has_edge(door, door_b):
                        self.G.add_edge(door, door_b)

        # Add edges between halls and door 0 of adjacent rooms
        for num in range(self.num_halls):  # Loop through list of hall links
            hall_str = 'h' + '%02d' % num  # Get the string for the specific hallway
            for rm_num in self.hall_room_links[num]:
                door_to_add = 'r' + '%02d' % rm_num + '_d00' + 'b'
                self.G.add_edge(hall_str, door_to_add)

        # Add edges for additional doors
        if self.hall_room_extra:
            self.G.add_edges_from(self.hall_room_extra)

        # Add edges between connected hallways
        if self.connected_halls:
            for (h0, h1) in self.connected_halls:
                hall_weight = int(h0[-2:]) + int(h1[-2:])
                self.G.add_edge(h0, h1)

        if self.connected_rooms:
            for i in range(self.num_rooms):
                if i != self.num_rooms - 1 and i != 1:
                    r0 = 'r' + '%02d' % i + '_d00' + 'b'
                    r1 = 'r' + '%02d' % (i + 1) + '_d00' + 'b'
                else:
                    # loop the final door back to the first door
                    r0 = 'r' + '%02d' % i + '_d00' + 'b'
                    r1 = 'r' + '%02d' % 0 + '_d00' + 'b'
                # print(r0, r1)
                self.G.add_edge(r0, r1)

    def _add_edge_weights(self):
        # Set up to add weights that are a custom interval of [0, 0]

        # nx.set_edge_attributes(self.G, , 'weight_no')
        for (n1, n2) in self.G.edges():
            # print(n1, n2)
            self.G[n1][n2]['weight_no'] = cust_interval(0)  # No humans present
            self.G[n1][n2]['weight_hum'] = cust_interval(0)  # Humans present
            self.G[n1][n2]['trav_data_no'] = []
            self.G[n1][n2]['trav_data_hum'] = []

            hum_cond = self.G.nodes[n1]['hum_cond']

            if hum_cond != self.G.nodes[n2]['hum_cond']:
                # Always use the exterior hall conditions over interior hall conditions
                if n1[0] == 'h':
                    hum_cond = self.G.nodes[n2]['hum_cond']
                elif n2[0] == 'h':
                    hum_cond = self.G.nodes[n1]['hum_cond']
                elif hum_cond > self.G.nodes[n2]['hum_cond']:
                    hum_cond = self.G.nodes[n2]['hum_cond']

            self.G[n1][n2]['hum_cond'] = hum_cond

    def _add_room_loc(self, node_info):
        # This will give the system x,y bounds for each room
        # This is used to choose where to autonomously drive next
        if node_info[1] is not 'x':
            self.G.add_node(node_info[0],
                            node_loc=[cust_interval(node_info[1], node_info[2]),
                                      cust_interval(node_info[3], node_info[4])],
                            hum_cond=node_info[-1])  # Adds the human presence condition

    def _add_door_loc(self, node_info):
        # Handle rooms first
        # Rooms will get one node in the door and one spanning the hall

        # Handles rooms then extra doors
        if 'd' not in node_info[0]:
            name_a = node_info[0] + '_d00a'
            name_b = node_info[0] + '_d00b'
        else:
            name_a = node_info[0] + 'a'
            name_b = node_info[0] + 'b'

        self.doors.append(name_a)
        self.doors.append(name_b)

        [xmin, xmax] = node_info[5:7]
        [ymin, ymax] = node_info[7:9]

        # 'a' is the node in the doorway
        xmin_a = xmin  # 5
        xmax_a = xmax  # 6
        ymin_a = ymin  # 16
        ymax_a = ymax  # 16

        # 'b' is the node just outside the doorway that spans the hall
        xmin_b = xmin  # 5
        xmax_b = xmax  # 6
        ymin_b = ymin  # 16
        ymax_b = ymax  # 16

        # Currently can't handle doors that are not parallel to an axis
        # Currently a door is listed as a line with no width
        # This adds width of 0.4m so it can detect when it's in that space
        # Vertical
        if xmin == xmax:

            # This is where things get very hand-coded (ugh)
            # TODO: Make this less world specific
            # Spans the hallway but doesn't overlap with the door node
            if xmin < 15:
                xmin_a = xmax_a - self.node_width
                xmin_b = xmax_a
                xmax_b = xmin_b + self.hall_width
            else:
                xmax_a = xmin_a + self.node_width
                xmax_b = xmin_a
                xmin_b = xmax_b - self.hall_width

            middle_y = (ymin_a + ymax_a) / 2
            ymin_b = middle_y - self.hall_node_width / 2
            ymax_b = middle_y + self.hall_node_width / 2

        # Horizontal
        elif ymin == ymax:

            # This is where things get very hand-coded (ugh)
            # TODO: Make this less world specific
            # Spans the hallway but doesn't overlap with the door node
            if ymin < 10:
                ymin_a = ymax_a - self.node_width
                ymin_b = ymax_a
                ymax_b = ymin_b + self.hall_width
            else:
                ymax_a = ymin_a + self.node_width
                ymax_b = ymin_a
                ymin_b = ymax_b - self.hall_width

            middle_x = (xmin_a + xmax_a) / 2
            xmin_b = middle_x - self.hall_node_width / 2
            xmax_b = middle_x + self.hall_node_width / 2

        else:
            print("something has gone wrong. One of these pairs should be ==")
            print(xmin, xmax, ymin, ymax)

        # Doorway
        self.G.add_node(name_a,
                        node_loc=[cust_interval(xmin_a, xmax_a),
                                  cust_interval(ymin_a, ymax_a)],
                        hum_cond=node_info[-1])  # Adds the human presence condition

        # Hallway node outside doorway
        self.G.add_node(name_b,
                        node_loc=[cust_interval(xmin_b, xmax_b),
                                  cust_interval(ymin_b, ymax_b)],
                        hum_cond=node_info[-1])  # Adds the human presence condition

        # Just in case I need to go back to this convention
        # self.G.nodes[name_a]['node_loc'] = [cust_interval(xmin_a, xmax_a),
        #                                     cust_interval(ymin_a, ymax_a)]
        # self.G.nodes[name_a]['hum_cond'] = node_info[-1]  # Adds the human presence condition

        # self.G.nodes[name_b]['node_loc'] = [cust_interval(xmin_b, xmax_b),
        #                                     cust_interval(ymin_b, ymax_b)]
        # self.G.nodes[name_b]['hum_cond'] = node_info[-1]  # Adds the human presence condition

        # print(name_a, self.nodes_dict[name_a])
        # print(name_b, self.nodes_dict[name_b])

    def _add_hall_loc(self, node_info):
        [xmin, xmax] = node_info[5:7]
        [ymin, ymax] = node_info[7:9]

        # Currently can't handle nodes that are not parallel to an axis
        # Currently nodes are listed as a line with no width
        # This adds width of 0.4m so it can detect when it's in that space
        # Vertical
        if xmin == xmax:
            xmin = xmin - self.node_width / 2
            xmax = xmax + self.node_width / 2

        # Horizontal
        elif ymin == ymax:
            ymin = ymin - self.node_width / 2
            ymax = ymax + self.node_width / 2

        else:
            print("something has gone wrong. One of these pairs should be ==")
            print(node_info[0], xmin, xmax, ymin, ymax)

        self.G.add_node(node_info[0],
                        node_loc=[cust_interval(xmin, xmax),
                                  cust_interval(ymin, ymax)],
                        hum_cond=node_info[-1])  # Adds the human presence condition

        # self.G.nodes[node_info[0]]['node_loc'] = [interval_cust(xmin, xmax),
        #                                  interval_cust(ymin, ymax),
        #                                  node_info[-1]]  # Adds the human presence condition

    def _add_excl_loc(self, node_info):
        self.G.add_node(node_info[0],
                        node_loc=[cust_interval(node_info[5], node_info[6]),
                                  cust_interval(node_info[7], node_info[8])],
                        hum_cond=node_info[-1])  # Adds the human presence condition

        # self.G.nodes[node_info[0]]['node_loc'] = [interval_cust.Interval(node_info[5], node_info[6]),
        #                                  interval_cust.Interval(node_info[7], node_info[8]),
        #                                  node_info[-1]]  # Adds the human presence condition

    def plot_graph(self):
        # Plot the graph
        pos = graphviz_layout(self.G)
        nx.draw_networkx(self.G, pos, with_labels=False)
        labels = nx.get_edge_attributes(self.G, 'weight')
        nx.draw_networkx_edge_labels(self.G, pos, edge_labels=labels)
        # nx.draw(self.G, with_labels=True)

        plt.show()

    def _draw_nodes_old(self):
        '''
        I believe this is now obsolete. I combined the node creation with the functions that add info about their
        physical locations.
        '''
        # Creates rooms in the format r## - numbers below 10 have a 0 appended in front for easier comparison
        self.rooms = ['r' + '%02d' % i for i in range(self.num_rooms)]

        # Creates one door with two nodes (node a & b)
        for rm in self.rooms:
            doors_a = rm + '_d00a'
            doors_b = rm + '_d00b'
            self.doors.append(doors_a)
            self.doors.append(doors_b)
            # self.G.add_edge(doors_a, doors_b)

        # Add extra doors to rooms with more than one
        if self.extra_doors:
            # Keep track of how many doors a room already has
            num_doors_per_rm = np.zeros(self.num_rooms)
            for room in self.extra_doors:
                rm = int(room[1:3])  # Get the room number
                num_doors_per_rm[rm] = num_doors_per_rm[rm] + 1  # Add additional door
                door_num = num_doors_per_rm[rm]  # Get door number
                new_nodes = ['r' + '%02d' % rm + '_d' + '%02d' % door_num + 'a',
                             'r' + '%02d' % rm + '_d' + '%02d' % door_num + 'b']  # Add two nodes to the door
                self.doors.extend(new_nodes)  # Add new nodes to the doors list
                self.G.add_edge(new_nodes[0], new_nodes[1])

        # Creates 5 hallways
        self.halls = ['h' + '%02d' % k for k in range(self.num_halls)]

        # Create nodes
        self.G.add_nodes_from(self.rooms)
        self.G.add_nodes_from(self.doors)
        self.G.add_nodes_from(self.halls)


def pickle_it(obj_to_pickle, file_path_and_name):
    with open(file_path_and_name, 'wb') as f:
        pickle.dump(obj_to_pickle, f)
    print("Saving information to file named {}".format(file_path_and_name))


def unpickle_it(filename):
    infile = open(filename, 'rb')
    unpickled = pickle.load(infile)
    infile.close()
    return unpickled


if __name__ == "__main__":
    num_rooms = 21
    num_halls = 4

    initial_pose = (1.0, 2.0)

    hall_width = 2.0
    node_width = 0.5
    hall_node_width = 1.0

    extra_doors = ['r01']
    extra_door_hall_links = [('r01_d01b', 'h00'), ('r01_d01b', 'h01'), ('r01_d01b', 'r02_d00b'),
                                  ('r01_d01b', 'r01_d00b'), ('r03_d00b', 'r05_d00b'), ('r07_d00b', 'r09_d00b'),
                                  ('r15_d00b', 'r17_d00b')]

    # Associates rooms (door 0) with the adjacent hall - additional doors handled further down
    hall0_rooms = [0, 1]
    hall1_rooms = [2]
    hall2_rooms = [10, 11, 12]
    hall3_rooms = [13, 14, 15]
    hall_door_links = [hall0_rooms, hall1_rooms, hall2_rooms, hall3_rooms]

    connected_halls = [('h00', 'h03'), ('h01', 'h02')]

    connected_rooms = True

    path_to_raw_param = '/home/toothless/workspaces/research_ws/src/hospital-world/scripts/STRUCT_hospital_v1_parameters_raw.txt'
    path_to_pickle = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/STRUCT_hospital_v1_param_pickle'

    # Set up a NetworkX graph of the building / hospital
    hospital = HospitalGraph(num_rooms, num_halls, extra_doors, hall_door_links, extra_door_hall_links,
                             connected_halls, connected_rooms, path_to_raw_param, initial_pose)

    # print(hospital.G.nodes['r00']['node_loc'][0])
    pickle_it(hospital.G, path_to_pickle)
    # hospital.plot_graph()

    # for (n1, n2) in hospital.G.edges():
    #     print(n1, n2)  #, hospital.G[n1][n2])
    #
    # for n in hospital.G.nodes():
    #     print(n, hospital.G.nodes[n])

